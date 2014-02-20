
/*
 * Copyright (C) 2010, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSCPP_MESSAGE_FACTORY_H
#define ROSCPP_MESSAGE_FACTORY_H

#include "ros/forwards.h"
#include "ros/time.h"
#include <ros/assert.h>
#include <ros/message_traits.h>
#include <ros/memfd_message.h>

#include <boost/type_traits/is_void.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_const.hpp>
#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>
#include <boost/interprocess/interprocess_fwd.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/container/deque.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>

namespace ros
{
  typedef boost::interprocess::managed_shared_memory::segment_manager segment_manager_type;
  typedef boost::interprocess::ros_allocator< void, ros::segment_manager_type> void_allocator_type;
  typedef boost::interprocess::deleter< void, ros::segment_manager_type>  void_deleter_type;
  typedef boost::interprocess::shared_ptr< void, ros::void_allocator_type, void_deleter_type > VoidIPtr;

/*
template<typename M>
static void DummyDeleter( M* ptr)
{
}
*/

class ShmemDeque
{
  public:

//  typedef boost::interprocess::deleter< void, ros::segment_manager_type>  deleter_type;
  typedef ShmemDeque* Ptr;
//  typedef boost::interprocess::shared_ptr< ShmemDeque, ros::void_allocator_type, deleter_type > IPtr;
//  typedef typename boost::interprocess::managed_shared_ptr< ShmemDeque, boost::interprocess::managed_shared_memory>::type IPtr;

  boost::interprocess::interprocess_mutex mutex_;

//  typedef boost::interprocess::deleter< void, ros::segment_manager_type>  VoidDeleterType;
//  typedef boost::interprocess::shared_ptr< void, ros::void_allocator_type, VoidDeleterType > VoidIPtr;
//  typedef boost::shared_ptr< void > VoidPtr;

  typedef typename ros::void_allocator_type::rebind<boost::uuids::uuid>::other allocator;
  boost::container::deque<boost::uuids::uuid, allocator> store_;

  boost::interprocess::interprocess_condition signal_;

  inline explicit ShmemDeque(const allocator& alloc)
    : store_(alloc) { };

  bool isEmpty()
  {
    return store_.empty();
  }

  void add(const boost::uuids::uuid& uuid)
  {
//    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);

    if (store_.size()>1000) {
//      fprintf(stderr,"deque is full, dropping msg\n");
      return;
    }

    store_.push_front(uuid);
    
  };

  const boost::uuids::uuid remove()
  {
//    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
//    fprintf(stderr,"popping message\n");
    boost::uuids::uuid uuid =store_.back();
    store_.pop_back();
    return uuid;
  };

};

class MessageFactory
{

private:
    static boost::shared_ptr<boost::interprocess::managed_shared_memory> segment;
public:

static typename ros::ShmemDeque::Ptr findDeque(const std::string& uuids)
{
  if (!segment) {
    ROS_DEBUG("Opening shmem segment");
    segment = boost::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "ros_test");
  }

  ROS_ASSERT(segment);

  typename ros::ShmemDeque::Ptr p = segment->find<ros::ShmemDeque>(uuids.c_str()).first;
  ROS_ASSERT(p);
  return p;
};

static typename ros::ShmemDeque::Ptr createDeque(const std::string& uuids)
{
  if (!segment) {
    ROS_DEBUG("Opening shmem segment");
    segment = boost::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "ros_test");
  }

  ROS_ASSERT(segment);

  typename ros::ShmemDeque::allocator alloc (segment->get_segment_manager());
  typename ros::ShmemDeque::Ptr sh_ptr = segment->construct<ros::ShmemDeque >(uuids.c_str())(alloc);
  ROS_ASSERT(sh_ptr);

  return sh_ptr;
};

template<typename M>
static M* createMessage()
{
  if (!segment)
    segment = boost::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "ros_test");

  boost::uuids::uuid uuid = boost::uuids::random_generator()();

  typename M::allocator alloc (segment->get_segment_manager());
  M* msg = segment->construct<M>(boost::uuids::to_string(uuid).c_str())(alloc);
  msg->uuid = uuid;
//  fprintf(stderr, "Created message with UUID %s\n", boost::uuids::to_string(msg->uuid).c_str());
//  typename M::Ptr p = typename M::Ptr(msg, &DummyDeleter<M>);
  return msg;
};

template<typename M>
static M* findMessage(const boost::uuids::uuid& uuid)
{
  if (!segment) {
    ROS_DEBUG("Opening shmem segment");
    segment = boost::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "ros_test");
  }

  ROS_ASSERT(segment);

  M* p = segment->find<M>(boost::uuids::to_string(uuid).c_str()).first;
  ROS_ASSERT(p);
  return p;
};

template<typename M>
static void destroyMessage(const M* msg)
{
  if (!segment) {
    ROS_DEBUG("Opening shmem segment");
    segment = boost::make_shared<boost::interprocess::managed_shared_memory>(boost::interprocess::open_only, "ros_test");
  }

  ROS_ASSERT(segment);
  ROS_ASSERT(msg);
  //fprintf(stderr," Destroying message: %s\n", msg->uuid);
  segment->destroy_ptr<M>(msg);
};


};

}

#endif // ROSCPP_MESSAGE_EVENT_H

/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#ifndef ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H
#define ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H

#include <typeinfo>

#include "common.h"
#include "ros/forwards.h"
#include "ros/parameter_adapter.h"
#include "ros/message_traits.h"
#include "ros/builtin_message_traits.h"
#include "ros/serialization.h"
#include "ros/message_event.h"
#include "ros/memfd_event.h"
#include <ros/static_assert.h>
#include <ros/memfd_message.h>

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/make_shared.hpp>

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/smart_ptr/deleter.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>

namespace ros
{

struct SubscriptionCallbackHelperDeserializeParams
{
  uint8_t* buffer;
  uint32_t length;
  boost::shared_ptr<M_string> connection_header;
  MemfdMessage::Ptr memfd_message;
  boost::uuids::uuid uuid;
};

struct ROSCPP_DECL SubscriptionCallbackHelperCallParams
{
  MessageEvent<void const> event;
  MessageEvent2<void const> event2;
};

/**
 * \brief Abstract base class used by subscriptions to deal with concrete message types through a common
 * interface.  This is one part of the roscpp API that is \b not fully stable, so overloading this class
 * is not recommended.
 */
class ROSCPP_DECL SubscriptionCallbackHelper
{
public:
  virtual ~SubscriptionCallbackHelper() {}
  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams&) = 0;
//  virtual void* deserializeShmem(const SubscriptionCallbackHelperDeserializeParams&) = 0;
  virtual void call(SubscriptionCallbackHelperCallParams& params) = 0;
  virtual const std::type_info& getTypeInfo() = 0;
  virtual bool isConst() = 0;
  virtual bool hasHeader() = 0;
};
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

/**
 * \brief Concrete generic implementation of
 * SubscriptionCallbackHelper for any normal message type.  Use
 * directly with care, this is mostly for internal use.
 */
template<typename P, typename Enabled = void>
class SubscriptionCallbackHelperT : public SubscriptionCallbackHelper
{
public:
  typedef ParameterAdapter<P> Adapter;
  typedef typename ParameterAdapter<P>::Message NonConstType;
  typedef typename ParameterAdapter<P>::Event Event;
  typedef typename boost::add_const<NonConstType>::type ConstType;
  typedef boost::shared_ptr<NonConstType> NonConstTypePtr;
  typedef boost::shared_ptr<ConstType> ConstTypePtr;

  static const bool is_const = ParameterAdapter<P>::is_const;

  typedef boost::function<void(typename Adapter::Parameter)> Callback;
  typedef boost::function<NonConstTypePtr()> CreateFunction;

  SubscriptionCallbackHelperT(const Callback& callback, 
			      const CreateFunction& create = DefaultMessageCreator<NonConstType>())
    : callback_(callback)
    , create_(create)
  { }

  void setCreateFunction(const CreateFunction& create)
  {
    create_ = create;
  }

  virtual bool hasHeader()
  {
     return message_traits::hasHeader<typename ParameterAdapter<P>::Message>();
  }

  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams& params)
  {
    namespace ser = serialization;

    NonConstTypePtr msg = create_();

    if (!msg)
    {
      ROS_DEBUG("Allocation failed for message of type [%s]", getTypeInfo().name());
      return VoidConstPtr();
    }

    ser::PreDeserializeParams<NonConstType> predes_params;
    predes_params.message = msg;
    predes_params.connection_header = params.connection_header;
    ser::PreDeserialize<NonConstType>::notify(predes_params);

    ser::IStream stream(params.buffer, params.length);
    ser::deserialize(stream, *msg);

    return VoidConstPtr(msg);
  }

  virtual void call(SubscriptionCallbackHelperCallParams& params)
  {
    // create typed copy from the untyped event given by the subscription queue
    Event event(params.event, create_);
    callback_(ParameterAdapter<P>::getParameter(event));
  }

  virtual const std::type_info& getTypeInfo()
  {
    return typeid(NonConstType);
  }

  virtual bool isConst()
  {
    return ParameterAdapter<P>::is_const;
  }

private:
  Callback callback_;
  CreateFunction create_;
};

/**
 * \brief Concrete generic implementation of
 * SubscriptionCallbackHelper for any normal message type.  Use
 * directly with care, this is mostly for internal use.
 */
template<typename P, typename Enabled = void>
class SubscriptionCallbackHelperT2 : public SubscriptionCallbackHelper
{
public:
  typedef ParameterAdapter<P> Adapter;
  typedef typename ParameterAdapter<P>::Message NonConstType;
  typedef typename ParameterAdapter<P>::Event Event;
  typedef typename ParameterAdapter<P>::Event2 Event2;
  typedef typename boost::add_const<NonConstType>::type ConstType;

  typedef boost::interprocess::managed_shared_memory::segment_manager segment_manager_type;
  typedef boost::interprocess::ros_allocator< void, segment_manager_type> void_allocator_type;
    
  typedef boost::interprocess::shared_ptr<NonConstType, void_allocator_type, boost::interprocess::deleter< NonConstType, segment_manager_type> > NonConstTypePtr;
  typedef boost::interprocess::shared_ptr<ConstType, void_allocator_type, boost::interprocess::deleter< ConstType, segment_manager_type> > ConstTypePtr;

  static const bool is_const = ParameterAdapter<P>::is_const;

  typedef boost::function<void(typename Adapter::Parameter2)> Callback2;
  typedef boost::function<NonConstTypePtr()> CreateFunction;
  typedef boost::function<NonConstTypePtr(const boost::uuids::uuid)> FindFunction;

  SubscriptionCallbackHelperT2(const Callback2& callback2,
			      const CreateFunction& create = DefaultShmemMessageCreator<NonConstType>())
    : callback2_(callback2)
    , create_(create)
  { }

  virtual bool hasHeader()
  {
     return message_traits::hasHeader<typename ParameterAdapter<P>::Message>();
  }

/*
  virtual NonConstTypePtr deserializeShmem(const SubscriptionCallbackHelperDeserializeParams& params)
  {
    // this is a ptr to a ipc shared_ptr in shmem
    NonConstTypePtr msg;// = create_shmem_(params.uuid);

    if (!msg)
    {
      ROS_DEBUG("Allocation failed for message of type [%s]", getTypeInfo().name());
      return NULL;
    }

    // ?? my header is empty, anyways
    //assignSubscriptionConnectionHeader(msg.get(), params.connection_header);

    return msg;
  }
*/

  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams& params)
  {
    namespace ser = serialization;

    //NonConstTypePtr msg = create_();

/*
    if (!msg)
    {
      ROS_DEBUG("Allocation failed for message of type [%s]", getTypeInfo().name());
      return VoidConstPtr();
    }

    ser::PreDeserializeParams<NonConstType> predes_params;
    predes_params.message = msg;
    predes_params.connection_header = params.connection_header;
    ser::PreDeserialize<NonConstType>::notify(predes_params);

    ser::IStream stream(params.buffer, params.length);
    ser::deserialize(stream, *msg);
*/
    return VoidConstPtr();
  }

  virtual void call(SubscriptionCallbackHelperCallParams& params)
  {
    // TODO: retrieve IPC shared ptr from segment manager and to cool stuff
    Event2 event(params.event2, create_);
    callback2_(ParameterAdapter<P>::getParameter2(event));
  }

  virtual const std::type_info& getTypeInfo()
  {
    return typeid(NonConstType);
  }

  virtual bool isConst()
  {
    return ParameterAdapter<P>::is_const;
  }

private:
  Callback2 callback2_;
  CreateFunction create_;
};

}

#endif // ROSCPP_SUBSCRIPTION_CALLBACK_HELPER_H

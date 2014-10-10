/**
  * Some kernel API define's. Since the memfd API is very new, they aren'T yet included in the system kernel headers.
  */

#ifndef _UAPI_LINUX_MEMFD_H
#define _UAPI_LINUX_MEMFD_H

/* flags for memfd_create(2) (unsigned int) */
#define MFD_CLOEXEC		0x0001U
#define MFD_ALLOW_SEALING	0x0002U

#ifndef __NR_memfd_create
#   define __NR_memfd_create 319
#endif

#ifndef F_ADD_SEALS
#define F_LINUX_SPECIFIC_BASE   1024
#define F_ADD_SEALS     (F_LINUX_SPECIFIC_BASE + 9)
#define F_GET_SEALS     (F_LINUX_SPECIFIC_BASE + 10)

#define F_SEAL_SEAL     0x0001  /* prevent further seals from being set */
#define F_SEAL_SHRINK   0x0002  /* prevent file from shrinking */
#define F_SEAL_GROW     0x0004  /* prevent file from growing */
#define F_SEAL_WRITE    0x0008  /* prevent writes */
#endif

#endif /* _UAPI_LINUX_MEMFD_H */

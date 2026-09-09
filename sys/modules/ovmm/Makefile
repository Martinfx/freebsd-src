.include <kmod.opts.mk>

KMOD=	ovmm

.PATH: ${SRCTOP}/sys/dev/ovmm ${SRCTOP}/sys/amd64/ovmm

SRCS=	ovmm.c		\
	ovmm_svm.c	\
	ovmm_support.S

SRCS+=	device_if.h bus_if.h

.include <bsd.kmod.mk>

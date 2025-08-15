#include <sys/cdefs.h>
#include "if_mt7622var.h"
#include <sys/kenv.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_arp.h>
#include <net/ethernet.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/if_vlan_var.h>

#include <net/bpf.h>

#include <machine/bus.h>
#include <machine/cache.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>
#include <vm/vm_param.h>
#include <vm/vm.h>
#include <vm/pmap.h>
#include <machine/pmap.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include "opt_platform.h"

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif
/*
#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mdio/mdio.h>
#include <dev/etherswitch/miiproxy.h>
#include "mdio_if.h"
#include "miibus_if.h"
*/

/* more specific and new models should go first */
static const struct ofw_compat_data rt_compat_data[] = {
        { "mediatek,mt7622-eth",    1 },
        { NULL,				0 }
};

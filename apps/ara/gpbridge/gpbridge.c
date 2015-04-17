/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/tsb/unipro.h>
#include <apps/greybus-utils/utils.h>

int bridge_main(int argc, char *argv[])
{
    tsb_gpio_register();
#ifdef CONFIG_BOARD_HAVE_DISPLAY
    display_init();
#endif

    enable_manifest("IID-1", NULL);
    gb_unipro_init();
    enable_cports();

#ifdef CONFIG_EXAMPLES_NSH
    printf("Calling NSH\n");
    return nsh_main(argc, argv);
#else
    return 0;
#endif
}


#include "platform.h"

#include "controller_node.h"

int main(int argc, char * argv[]) {
  platform_init();

  controller_node_main(argc, argv);

  platform_exit();

  return 0;
}
#include <pick/pick.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_node");

  ros::AsyncSpinner spinner(2);  // Multi-threaded callback handling
  spinner.start();

  Pick pick;
  pick.init();

  ros::Rate loop_rate(10);
  while (ros::ok())
  { 
    pick.update();  // 👈 Call update in the main loop
    loop_rate.sleep();
  }

  ros::shutdown();  // 👈 Properly call shutdown
  return 0;
}

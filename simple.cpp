#include <stdio.h>
#include <dirent.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
using namespace std;

int main(){
    rosbag::Bag bag;
    bag.open("./imu.bag");
    cout << bag.getSize() << endl;
    cout << bag.getFileName() << endl;
    if(!bag.isOpen()){
        perror("open bag fault in : ");
        return -1;
    }
    
    rosbag::View view(bag, rosbag::TopicQuery("/points_raw"));
    int num = view.size();
    printf("viwe num = %d", num);
}

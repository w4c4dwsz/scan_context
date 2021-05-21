#include <iostream>
#include <pcl/pcl_base.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include "Scancontext.h"

static int
dirSelect (const struct dirent *unused)
{
    if(unused->d_name[0] == '.'){
        return 0;
    }
    
  return 1;
}

void
getFilenames(std::vector<std::string> filenameVector){
    std::string currWorkDir(getcwd(nullptr, 0));
    struct dirent ** eps;
    int fileNum = scandir(currWorkDir.c_str(), &eps, dirSelect, alphasort);
    if(fileNum = -1){
        puts("In function getFIlenames, scandir fault\n");
        perror("The following error occurred");
    }
    for(int i = 0; i < fileNum; i++){
        
    }
}
int main(int argc, char** argv) {
    std::cout << "Hello, world!\n";

    // set  savePCDDirectory
    std::string savePCDDirectory = "/home/alinx/Downloads/SCLOAM/20200129";
    if(argc == 2) {
        savePCDDirectory = argv[1];
    }

    // set saveSCDDirectory and saveNodePCDDirectory
    std::string saveSCDDirectory = savePCDDirectory + "SCDs/";
    std::string saveNodePCDDirectory = savePCDDirectory + "Scans/";

    std::cout<< "savePCDDirectory=" << savePCDDirectory << endl;
    std::cout<< "saveSCDDirectory=" << saveSCDDirectory << endl;
    std::cout<< "saveNodePCDDirectory=" << saveNodePCDDirectory << endl;

    if(chdir(savePCDDirectory.c_str()) != 0){
        printf("change work space to savePCDDirectory fault\n");
        perror("The following error occurred");
        return -1;
    }
    printf("work dir = %s", getcwd(nullptr, 0));

    struct dirent ** eps;
    
    int fileNum = scandir("./", &eps, dirSelect, alphasort);
    if(fileNum == -1){
        printf("get file list falut\n");
        return -1;
    }
    for(int i = 0; i < fileNum; i++){
        printf("%s\n", eps[i]->d_name);
    }

    SCManager scManager;
     

    
    return 0;
}

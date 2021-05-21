#include <stdio.h>
#include <dirent.h>


static int
one (const struct dirent *unused)
{
    if(unused->d_name[0] == '.'){
        return 0;
    }
    
  return 1;
}

int
main (void)
{
  DIR* dir;
  dir = opendir("./");
  struct dirent * ep;
  ep = readdir(dir);
  puts(ep->d_name);
  ep = readdir(dir);
  puts(ep->d_name);
  return 0;
}

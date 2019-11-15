
#include "strategy.h"

int main(int argc, char** argv){
    srand(time(NULL));

    Strategy strategy;

    while(true){
        
        strategy.loop();
        
    //    send_commands();
      //  send_debug();
    }

    return 0;
}
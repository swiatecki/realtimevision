#include <iostream>

extern "C" {
  #include "wrapper_test.h" //a C header, so wrap it in extern "C" 
}

int main(int argc, char** argv){

writeTest();

}
#include "exClass.h"

exClass::exClass(int var_num){
    var = var_num;
}

int exClass::exPublicFunc(int num){
    int temp = exPrivateFunc(num);
    return temp*2;
}

int exClass::exPrivateFunc(int num){
    return num+1;
}
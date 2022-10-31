#include <math.h>
// any other includes


#define THIS_IS_A_NUMBER 10
// any other defines

class exClass{
    public:
        //constructor
        exClass(int);

        //public functions
        int exPublicFunc(int);

    private:
        //private variable declarations
        int var;

        //private functions that can only be used within this class
        int exPrivateFunc(int);
};


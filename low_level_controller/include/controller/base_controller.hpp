#ifndef BASE_CONTROLLER
#define BASE_CONTROLLER

class BaseController {
    virtual void init(void) = 0;
    virtual void update(void) = 0;
    virtual void stop(void) = 0;
};

#endif//BASE_CONTROLLER
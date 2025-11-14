#ifndef __FLAP_H_INCLUDED__
#define __FLAP_H_INCLUDED__

class Flap
{
public:
    void open();
    void close();
    bool isOpen();

private:
    bool isOpen = false;
};

#endif
#ifndef DIMMER_H
#define DIMMER_H

class Dimmer
{
public:
    Dimmer(uint8_t nbChannels = 3);
    void update(void);
};


#endif /*DIMMER_H */
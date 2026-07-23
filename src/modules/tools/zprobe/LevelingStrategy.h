/*
 * A strategy called from ZProbe to handle a strategy for leveling
 * examples are delta calibration, three point bed leveling, z height map
 */

#ifndef _LEVELINGSTRATEGY
#define _LEVELINGSTRATEGY

class ZProbe;
class Gcode;

class LevelingStrategy
{
public:
    LevelingStrategy(ZProbe* zprobe) : zprobe(zprobe){};
    virtual ~LevelingStrategy(){};
    virtual bool handleGcode(Gcode* gcode)= 0;
    virtual bool handleConfig()= 0;
    // Called after config_cache_clear() so strategies can safely allocate on the main heap
    virtual void after_config_cache_clear() {}

protected:
    ZProbe *zprobe;

};
#endif

#ifndef REMOTE_H
#define REMOTE_H

#include "Shared.h"
#include "DataStore.h"
#include <IRremote.h>
#include <functional>

class Remote {
public:
    Remote(DataStore& data);
    void begin();
    void handle();

private:
    void handleIRCommand();
    DataStore& data;
};

#endif // REMOTE_H
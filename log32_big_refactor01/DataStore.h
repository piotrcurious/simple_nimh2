#ifndef DATASTORE_H
#define DATASTORE_H

#include "Shared.h"

class DataStore {
public:
    DataStore();

    // State Management
    AppState app_state;
};

#endif // DATASTORE_H
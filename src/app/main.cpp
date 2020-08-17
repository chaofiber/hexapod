#include "app.h"

#include <iostream>
#include <gui/model.h>

int main() {

    App app;
	app.setCallbacks();
	app.run();

	return 0;
}

#include <TeensyVector.h>
#include "src/SO/SO.h"

using namespace Cyberpod;

SO so;

void setup()
{
  delay(3000);
  so.init();
}

void loop()
{
  so.update();
}

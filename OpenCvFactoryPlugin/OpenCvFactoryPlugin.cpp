#include "OpenCvFactoryPlugin.h"

#if QT_VERSION < 0x050000
Q_EXPORT_PLUGIN2(OpenCvFactoryPlugin, OpenCvFactoryPlugin)
#endif // QT_VERSION < 0x050000


int OpenCvFactoryPlugin::nextTag() {
  static int sequence = 0;
  return sequence++;
}


#ifndef OBJECTMODEL_GLOBAL_H
#define OBJECTMODEL_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(OBJECTMODEL_LIBRARY)
#  define OBJECTMODELSHARED_EXPORT Q_DECL_EXPORT
#else
#  define OBJECTMODELSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // OBJECTMODEL_GLOBAL_H

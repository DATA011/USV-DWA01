#ifndef CUSVMODEL_GLOBAL_H
#define CUSVMODEL_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CUSVMODEL_LIBRARY)
#  define CUSVMODEL_EXPORT Q_DECL_EXPORT
#else
#  define CUSVMODEL_EXPORT Q_DECL_IMPORT
#endif

#endif // CUSVMODEL_GLOBAL_H

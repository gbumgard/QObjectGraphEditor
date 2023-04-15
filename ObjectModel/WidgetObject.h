#ifndef ABSTRACTOBJECT_H
#define ABSTRACTOBJECT_H

#include <QObject>
#include <type_traits>
#include <QSlider>

#if 0
template<class BaseClass>
class ObjectBase : public BaseClass
{

    Q_OBJECT

protected:

    explicit ObjectBase(QObject* parent)
        : BaseClass(parent)
    {
        static_assert(std::is_base_of<QObject, BaseClass>::value);
    }

    virtual void update() = 0;


};

template<class BaseClass>
class WidgetBase : public ObjectBase<BaseClass>
{
    Q_OBJECT

protected:
    explicit WidgetBase(QObject *parent)
        : ObjectBase<BaseClass>(parent)
    {
        static_assert(std::is_base_of<QWidget, WidgetBase>::value);
    }

};

class Slider : public WidgetBase<QSlider>
{

};

class Filter : public ObjectBase<QObject> {

};

#endif

#endif // ABSTRACTOBJECT_H

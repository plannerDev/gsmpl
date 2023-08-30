#pragma once

#include <memory>

#define GSMPL_DECLARE_PTR(Name, Type)                   \
    typedef std::shared_ptr<Type> Name##Ptr;            \
    typedef std::shared_ptr<const Type> Name##ConstPtr; \
    typedef std::unique_ptr<Type> Name##UPtr;

#define GSMPL_CLASS_FORWARD(C) \
    class C;                   \
    GSMPL_DECLARE_PTR(C, C)

#define GSMPL_STRUCT_FORWARD(C) \
    struct C;                   \
    GSMPL_DECLARE_PTR(C, C)

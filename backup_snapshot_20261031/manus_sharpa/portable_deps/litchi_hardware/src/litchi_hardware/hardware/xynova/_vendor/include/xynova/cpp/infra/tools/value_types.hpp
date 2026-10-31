#ifndef XYNOVA_CPP_VALUE_TYPES_HPP
#define XYNOVA_CPP_VALUE_TYPES_HPP

/**
 * @file value_types.hpp
 * @brief C++ strongly typed value wrappers for Xynova SDK.
 *
 * This file provides C++ wrappers for C API value types defined in
 * @ref xynova/c/value_types.h.
 *
 * Features:
 * - Automatic C++ type to XynovaDataType mapping.
 * - Strongly typed scalar and array value containers.
 * - C API conversion support.
 * - Identifier value wrappers.
 *
 * Example:
 * @code
 * using namespace xynova::infra::tools;
 *
 * Value<float> angle(30.0f);
 * Value<float, 3> position({1.0f, 2.0f, 3.0f});
 *
 * auto c_value = angle.toC();
 * @endcode
 *
 * @version 1.0.0
 * @date 2026-07-17
 */

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "xynova/c/value_types.h"

namespace xynova {
namespace infra {
namespace tools {

/**
 * @brief Mapping between C++ type and Xynova data type.
 *
 * Specializations are generated for supported SDK value types.
 *
 * @tparam T C++ type.
 */
template<typename T>
struct DataTypeTraits
{
    static constexpr uint8_t value = XYNOVA_DATA_TYPE_NONE;
};

/**
 * @brief Define a C++ type to XynovaDataType mapping.
 *
 * @param cpp_type C++ type.
 * @param c_type Xynova data type identifier.
 */
#define XYNOVA_DEFINE_DATA_TYPE_TRAIT(cpp_type, c_type) \
template<> \
struct DataTypeTraits<cpp_type> \
{ \
    static constexpr uint8_t value = c_type; \
};

XYNOVA_DEFINE_DATA_TYPE_TRAIT(bool, XYNOVA_DATA_TYPE_BOOL)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(uint8_t, XYNOVA_DATA_TYPE_U8)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(int8_t, XYNOVA_DATA_TYPE_S8)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(uint16_t, XYNOVA_DATA_TYPE_U16)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(int16_t, XYNOVA_DATA_TYPE_S16)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(uint32_t, XYNOVA_DATA_TYPE_U32)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(int32_t, XYNOVA_DATA_TYPE_S32)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(uint64_t, XYNOVA_DATA_TYPE_U64)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(int64_t, XYNOVA_DATA_TYPE_S64)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(float, XYNOVA_DATA_TYPE_FLOAT)
XYNOVA_DEFINE_DATA_TYPE_TRAIT(double, XYNOVA_DATA_TYPE_DOUBLE)

#undef XYNOVA_DEFINE_DATA_TYPE_TRAIT

/**
 * @brief Store C++ value into C API union.
 *
 * @tparam T Value type.
 * @param dst Destination C union.
 * @param src Source C++ value.
 */
template<typename T>
inline void assignValue(XynovaValue &dst, const T &src)
{
    static_assert(DataTypeTraits<T>::value != XYNOVA_DATA_TYPE_NONE,
                  "Unsupported value type");

    if constexpr (std::is_same_v<T, bool>)
        dst.u8 = src ? 1 : 0;
    else if constexpr (std::is_same_v<T, uint8_t>)
        dst.u8 = src;
    else if constexpr (std::is_same_v<T, int8_t>)
        dst.s8 = src;
    else if constexpr (std::is_same_v<T, uint16_t>)
        dst.u16 = src;
    else if constexpr (std::is_same_v<T, int16_t>)
        dst.s16 = src;
    else if constexpr (std::is_same_v<T, uint32_t>)
        dst.u32 = src;
    else if constexpr (std::is_same_v<T, int32_t>)
        dst.s32 = src;
    else if constexpr (std::is_same_v<T, uint64_t>)
        dst.u64 = src;
    else if constexpr (std::is_same_v<T, int64_t>)
        dst.s64 = src;
    else if constexpr (std::is_same_v<T, float>)
        dst.f32 = src;
    else if constexpr (std::is_same_v<T, double>)
        dst.f64 = src;
}

/**
 * @brief Read C API union into C++ value.
 *
 * @tparam T Value type.
 * @param src Source C union.
 *
 * @return Converted C++ value.
 */
template<typename T>
inline T readValue(const XynovaValue &src)
{
    static_assert(DataTypeTraits<T>::value != XYNOVA_DATA_TYPE_NONE,
                  "Unsupported value type");

    if constexpr (std::is_same_v<T, bool>)
        return src.u8 != 0;
    else if constexpr (std::is_same_v<T, uint8_t>)
        return src.u8;
    else if constexpr (std::is_same_v<T, int8_t>)
        return src.s8;
    else if constexpr (std::is_same_v<T, uint16_t>)
        return src.u16;
    else if constexpr (std::is_same_v<T, int16_t>)
        return src.s16;
    else if constexpr (std::is_same_v<T, uint32_t>)
        return src.u32;
    else if constexpr (std::is_same_v<T, int32_t>)
        return src.s32;
    else if constexpr (std::is_same_v<T, uint64_t>)
        return src.u64;
    else if constexpr (std::is_same_v<T, int64_t>)
        return src.s64;
    else if constexpr (std::is_same_v<T, float>)
        return src.f32;
    else if constexpr (std::is_same_v<T, double>)
        return src.f64;
}

/**
 * @brief Strongly typed value wrapper.
 *
 * This class provides a C++ abstraction over:
 * - XynovaTypedValue
 * - XynovaTypedValueArray2
 * - XynovaTypedValueArray3
 * - XynovaTypedValueArray4
 *
 * @tparam T Element type.
 * @tparam N Number of elements. Range: 1~4.
 */
template<typename T, size_t N = 1>
class Value
{
public:
    static_assert(N >= 1 && N <= 6,
                "Value supports 1 to 6 elements only");

    using Storage =
        std::conditional_t<N == 1, T, std::array<T, N>>;

public:
    /**
     * @brief Default constructor.
     */
    Value() = default;

    /**
     * @brief Construct from value.
     *
     * @param value Initial value.
     */
    explicit Value(const Storage &value)
        : i_value(value)
    {
    }

    /**
     * @brief Set value.
     *
     * @param value New value.
     */
    void setValue(const Storage &value)
    {
        i_value = value;
    }

    /**
     * @brief Get constant value.
     *
     * @return Constant reference.
     */
    const Storage &value() const noexcept
    {
        return i_value;
    }

    /**
     * @brief Get mutable value.
     *
     * @return Mutable reference.
     */
    Storage &value() noexcept
    {
        return i_value;
    }

    /**
     * @brief Convert from C API value.
     *
     * The input type must match the template element count.
     *
     * @tparam CValue C API value type.
     *
     * @param value Source C API value.
     *
     * @return true if conversion succeeds, false otherwise.
     */
    template<typename CValue>
    bool fromC(const CValue &value)
    {
        if constexpr (std::is_same_v<CValue, XynovaTypedValue>)
        {
            static_assert(N == 1,
                        "Scalar value requires N == 1");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            i_value = readValue<T>(value.value);

            return true;
        }
        else if constexpr (std::is_same_v<CValue, XynovaTypedValueArray2>)
        {
            static_assert(N == 2,
                        "Array2 value requires N == 2");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            for (size_t i = 0; i < N; ++i)
                i_value[i] = readValue<T>(value.values[i]);

            return true;
        }
        else if constexpr (std::is_same_v<CValue, XynovaTypedValueArray3>)
        {
            static_assert(N == 3,
                        "Array3 value requires N == 3");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            for (size_t i = 0; i < N; ++i)
                i_value[i] = readValue<T>(value.values[i]);

            return true;
        }
        else if constexpr (std::is_same_v<CValue, XynovaTypedValueArray4>)
        {
            static_assert(N == 4,
                        "Array4 value requires N == 4");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            for (size_t i = 0; i < N; ++i)
                i_value[i] = readValue<T>(value.values[i]);

            return true;
        }
        else if constexpr (std::is_same_v<CValue, XynovaTypedValueArray5>)
        {
            static_assert(N == 5,
                        "Array5 value requires N == 5");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            for (size_t i = 0; i < N; ++i)
                i_value[i] = readValue<T>(value.values[i]);

            return true;
        }
        else if constexpr (std::is_same_v<CValue, XynovaTypedValueArray6>)
        {
            static_assert(N == 6,
                        "Array6 value requires N == 6");

            if (value.type != DataTypeTraits<T>::value)
                return false;

            for (size_t i = 0; i < N; ++i)
                i_value[i] = readValue<T>(value.values[i]);

            return true;
        }
        else
        {
            static_assert(sizeof(CValue) == 0,
                        "Unsupported C API value type");

            return false;
        }
    }

    /**
     * @brief Convert to C API value.
     *
     * @return C API value object.
     */
    auto toC() const
    {
        if constexpr (N == 1)
        {
            XynovaTypedValue result{};

            result.type = DataTypeTraits<T>::value;
            assignValue(result.value, i_value);

            return result;
        }
        else if constexpr (N == 2)
        {
            XynovaTypedValueArray2 result{};

            result.type = DataTypeTraits<T>::value;

            for (size_t i = 0; i < N; ++i)
                assignValue(result.values[i], i_value[i]);

            return result;
        }
        else if constexpr (N == 3)
        {
            XynovaTypedValueArray3 result{};

            result.type = DataTypeTraits<T>::value;

            for (size_t i = 0; i < N; ++i)
                assignValue(result.values[i], i_value[i]);

            return result;
        }
        else if constexpr (N == 4)
        {
            XynovaTypedValueArray4 result{};

            result.type = DataTypeTraits<T>::value;

            for (size_t i = 0; i < N; ++i)
                assignValue(result.values[i], i_value[i]);

            return result;
        }
        else if constexpr (N == 5)
        {
            XynovaTypedValueArray5 result{};

            result.type = DataTypeTraits<T>::value;

            for (size_t i = 0; i < N; ++i)
                assignValue(result.values[i], i_value[i]);

            return result;
        }
        else if constexpr (N == 6)
        {
            XynovaTypedValueArray6 result{};

            result.type = DataTypeTraits<T>::value;

            for (size_t i = 0; i < N; ++i)
                assignValue(result.values[i], i_value[i]);

            return result;
        }
    }

private:
    Storage i_value{};
};


/**
 * @brief Strongly typed identifier value wrapper.
 *
 * Wraps an identifier together with a typed value.
 *
 * @tparam T Element type.
 * @tparam N Number of elements. Range: 1~6.
 */
template<typename T, size_t N = 1>
class IdValue
{
public:
    using ValueType = Value<T, N>;
    using Storage = typename ValueType::Storage;

public:
    /**
     * @brief Default constructor.
     */
    IdValue() = default;

    /**
     * @brief Construct identifier value.
     *
     * @param id Identifier.
     * @param value Initial value.
     */
    IdValue(uint16_t id, const Storage &value)
        : i_id(id),
          i_value(value)
    {
    }

    /**
     * @brief Set identifier.
     *
     * @param id Identifier value.
     */
    void setId(uint16_t id) noexcept
    {
        i_id = id;
    }

    /**
     * @brief Set contained value.
     *
     * @param value New value.
     */
    void setValue(const Storage &value)
    {
        i_value.setValue(value);
    }

    /**
     * @brief Get identifier.
     *
     * @return Identifier value.
     */
    uint16_t id() const noexcept
    {
        return i_id;
    }

    /**
     * @brief Get constant value wrapper.
     *
     * @return Constant reference.
     */
    const ValueType &value() const noexcept
    {
        return i_value;
    }

    /**
     * @brief Get mutable value wrapper.
     *
     * @return Mutable reference.
     */
    ValueType &value() noexcept
    {
        return i_value;
    }

    /**
     * @brief Convert from C API identifier value.
     *
     * @tparam CValue C API identifier value type.
     *
     * @param value Source C API value.
     *
     * @return true if conversion succeeds, false otherwise.
     */
    template<typename CValue>
    bool fromC(const CValue &value)
    {
        if (!i_value.fromC(value.data))
            return false;

        i_id = value.id;

        return true;
    }

    /**
     * @brief Convert to C API identifier value.
     *
     * @return C API identifier value object.
     */
    auto toC() const
    {
        if constexpr (N == 1)
        {
            XynovaIdValue result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else if constexpr (N == 2)
        {
            XynovaIdValueArray2 result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else if constexpr (N == 3)
        {
            XynovaIdValueArray3 result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else if constexpr (N == 4)
        {
            XynovaIdValueArray4 result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else if constexpr (N == 5)
        {
            XynovaIdValueArray5 result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else if constexpr (N == 6)
        {
            XynovaIdValueArray6 result{};

            result.id = i_id;
            result.data = i_value.toC();

            return result;
        }
        else
        {
            static_assert(N <= 6,
                        "Unsupported IdValue size");
        }
    }

private:
    uint16_t i_id{0};
    ValueType i_value;
};

} // namespace tools
} // namespace infra
} // namespace xynova

#endif /* XYNOVA_CPP_VALUE_TYPES_HPP */
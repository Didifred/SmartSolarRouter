
#ifndef UTILS_H
#define UTILS_H

#include <vector>

class Utils
{

  /**
   * @class Utils
   * @brief Utility class providing various helper functions.
   */

public:
  Utils() = delete; // Prevent instantiation of this class
  ~Utils() = delete;

  /**
   * @brief Initializes the Logger module.
   */
  static void checkEspFlash(void);

  /**
   * @brief Initialize a hardware timer to call a user function at a specified frequency.
   * @param gridFrequency The frequency in Hz.
   * @param userFunc The user function to be called by the timer interrupt.
   */
  static void initHwTimer(uint8_t frequency, timercallback userFunc);

  /**
   * @brief Disables the hardware timer interrupts.
   */
  static void disableHwTimer(void);

  /**
   * @brief Enable the hardware timer interrupts.
   */
  static void enableHwTimer(void);
};

/**
 * @brief Template-based Bitset class using std::array for storage
 *
 * @tparam N Number of bits in the bitfield
 */
template <size_t N>
class Bitset
{
public:
  /**
   * @brief Construct a new Bitset object
   *        Initialize bitset with N bits, all set to 0
   */
  Bitset() : m_data{} {} // Zero-initialization

  /**
   * @brief Copy constructor
   *
   * @param other Bitset to copy from
   */
  Bitset(const Bitset &other) : m_data(other.m_data) {}

  /**
   * @brief Copy assignment operator
   *
   * @param other Bitfield to copy from
   * @return Bitfield& Reference to this object
   */
  Bitset &operator=(const Bitset &other)
  {
    if (this != &other)
    {
      m_data = other.m_data;
    }
    return *this;
  }

  /**
   * @brief Set bit at position to 1
   *
   * @param pos Position of the bit (0 to N-1)
   */
  void set(size_t pos)
  {
    if (pos < N)
    {
      size_t byteIndex = pos / 8;
      size_t bitIndex = pos % 8;
      m_data[byteIndex] |= (1 << bitIndex);
    }
  }

  /**
   * @brief Clear bit at position to 0
   *
   * @param pos Position of the bit (0 to N-1)
   */
  void clear(size_t pos)
  {
    if (pos < N)
    {
      size_t byteIndex = pos / 8;
      size_t bitIndex = pos % 8;
      m_data[byteIndex] &= ~(1 << bitIndex);
    }
  }

  /**
   * @brief Get value of bit at position
   *
   * @param pos Position of the bit (0 to N-1)
   * @return true/false if bit is 1/0
   */
  bool get(size_t pos) const
  {
    bool value = false;
    if (pos < N)
    {
      size_t byteIndex = pos / 8;
      size_t bitIndex = pos % 8;
      value = ((m_data[byteIndex] & (1 << bitIndex)) != 0);
    }
    return value;
  }

  /**
   * @brief Get total number of bits
   *
   * @return size_t Number of bits
   */
  constexpr size_t size() const
  {
    return N;
  }

  /**
   * @brief Clear all bits to 0
   */
  void clearAll()
  {
    m_data.fill(0);
  }

  /**
   * @brief Set all bits to 1
   */
  void setAll()
  {
    m_data.fill(0xFF);
  }

  /**
   * @brief Convert bitfield to string representation
   *
   * @return String String of '0' and '1' characters
   */
  String toString() const
  {
    String result;

    result.reserve(N);

    for (size_t i = 0; i < N; ++i)
    {
      result += (get(i) ? '1' : '0');
    }

    return result;
  }

private:
  static constexpr size_t m_numBytes = (N + 7) / 8; // Ceiling division
  std::array<uint8_t, m_numBytes> m_data;
};

#endif /* UTILS_H */

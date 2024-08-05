/*
 * BLEValue.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: kolban
 */

#include "BLEValue.h"
#include "Arduino.h"

BLEValue::BLEValue()
{
	m_accumulation = "";
	m_value = NULL;
	m_length = 0;
	m_readOffset = 0;
} // BLEValue

/**
 * @brief Add a message part to the accumulation.
 * The accumulation is a growing set of data that is added to until a commit or cancel.
 * @param [in] part A message part being added.
 */
void BLEValue::addPart(std::string part)
{
	m_accumulation += part;
} // addPart

/**
 * @brief Add a message part to the accumulation.
 * The accumulation is a growing set of data that is added to until a commit or cancel.
 * @param [in] pData A message part being added.
 * @param [in] length The number of bytes being added.
 */
void BLEValue::addPart(uint8_t *pData, size_t length)
{
	m_accumulation += std::string((char *)pData, length);
} // addPart

/**
 * @brief Cancel the current accumulation.
 */
void BLEValue::cancel()
{
	m_accumulation = "";
	m_readOffset = 0;
} // cancel

/**
 * @brief Commit the current accumulation.
 * When writing a value, we may find that we write it in "parts" meaning that the writes come in in pieces
 * of the overall message.  After the last part has been received, we may perform a commit which means that
 * we now have the complete message and commit the change as a unit.
 */
void BLEValue::commit()
{
	// If there is nothing to commit, do nothing.
	if (m_accumulation.length() == 0)
		return;
	setValue(m_accumulation);
	m_accumulation = "";
	m_readOffset = 0;
} // commit

/**
 * @brief Get a pointer to the data.
 * @return A pointer to the data.
 */
uint8_t *BLEValue::getData()
{
	return (uint8_t *)m_value;
}

/**
 * @brief Get the length of the data in bytes.
 * @return The length of the data in bytes.
 */
size_t BLEValue::getLength()
{
	return m_length;
} // getLength

/**
 * @brief Get the read offset.
 * @return The read offset into the read.
 */
uint16_t BLEValue::getReadOffset()
{
	return m_readOffset;
} // getReadOffset

/**
 * @brief Get the current value.
 */
std::string BLEValue::getValue()
{
	if (m_value != NULL && m_length != 0)
	{
		return std::string((char *)m_value, m_length);
	}
	return "";
} // getValue

/**
 * @brief Set the read offset
 * @param [in] readOffset The offset into the read.
 */
void BLEValue::setReadOffset(uint16_t readOffset)
{
	m_readOffset = readOffset;
} // setReadOffset

/**
 * @brief Set the current value.
 */
void BLEValue::setValue(std::string value)
{
	if (m_value != NULL)
	{
		free(m_value);
		m_value = NULL;
	}
	m_value = (uint8_t *)malloc(sizeof(uint8_t) * value.length());
	if (m_value != NULL)
	{
		memcpy(m_value, value.data(), value.length());
		m_length = value.length();
	}
	else
	{
		m_length = 0;
	}
} // setValue

/**
 * @brief Set the current value.
 * @param [in] pData The data for the current value.
 * @param [in] The length of the new current value.
 */
void BLEValue::setValue(uint8_t *pData, size_t length)
{
	if (m_value != NULL)
	{
		free(m_value);
		m_value = NULL;
	}
	m_value = (uint8_t *)malloc(sizeof(uint8_t) * length);
	if (m_value != NULL)
	{
		memcpy(m_value, pData, length);
		m_length = length;
	}
	else
	{
		m_length = 0;
	}
} // setValue
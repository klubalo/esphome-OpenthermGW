#include "openthermgw.h"

OpenTherm *esphome::openthermgw::OpenthermGW::mOT;
OpenTherm *esphome::openthermgw::OpenthermGW::sOT;
esphome::switch_::Switch *esphome::openthermgw::OpenthermGW::switch_dhw_pump_override;
esphome::switch_::Switch *esphome::openthermgw::OpenthermGW::switch_dhw_pump_override_mode;
std::map<int, std::vector<esphome::openthermgw::OpenthermGW::AcmeSensorInfo *> *> esphome::openthermgw::OpenthermGW::acme_sensor_map;
std::map<int, std::vector<esphome::openthermgw::OpenthermGW::AcmeBinarySensorInfo *> *> esphome::openthermgw::OpenthermGW::acme_binary_sensor_map;
std::map<int, std::vector<esphome::openthermgw::OpenthermGW::OverrideBinarySwitchInfo *> *> esphome::openthermgw::OpenthermGW::override_binary_switch_map;
std::map<int, std::vector<esphome::openthermgw::OpenthermGW::OverrideNumericSwitchInfo *> *> esphome::openthermgw::OpenthermGW::override_numeric_switch_map;

namespace esphome {
namespace openthermgw {

    void OpenthermGW::set_master_in_pin(uint8_t pin) { master_in_pin_ = pin; }
    void OpenthermGW::set_master_out_pin(uint8_t pin) { master_out_pin_ = pin; }
    void OpenthermGW::set_slave_in_pin(uint8_t pin) { slave_in_pin_ = pin; }
    void OpenthermGW::set_slave_out_pin(uint8_t pin) { slave_out_pin_ = pin; }
    
    OpenthermGW::OpenthermGW(): PollingComponent(60000)
    {
        mOT = nullptr;
        sOT = nullptr;

        switch_dhw_pump_override            = nullptr;
        switch_dhw_pump_override_mode       = nullptr;
    }

    void IRAM_ATTR OpenthermGW::mHandleInterrupt()
    {
        mOT->handleInterrupt();
    }

    void IRAM_ATTR OpenthermGW::sHandleInterrupt()
    {
        sOT->handleInterrupt();
    }

    void OpenthermGW::processRequest(unsigned long request, OpenThermResponseStatus status)
    {
        unsigned char requestDataID = mOT->getDataID(request);
        unsigned char requestMessageType = mOT->getMessageType(request);
        unsigned short requestDataValue = request & 0xffff;
        bool invalid_data = false;

        ESP_LOGD(TAG, "Opentherm request [MessageType: %s, DataID: %d, Data: %x, status %s]", mOT->messageTypeToString(mOT->getMessageType(request)), requestDataID, requestDataValue, sOT->statusToString(status));
        
        // Check validity of the original request
        if(status != OpenThermResponseStatus::SUCCESS || mOT->parity(request))
        {
            if (mOT->getMessageType(request) == INVALID_DATA)
            {
                // A data item to be sent by the master may be invalid in a particular application, but may still require to be sent.
                // In this case the master may use the message type �data invalid�.
                ESP_LOGD(TAG, "Opentherm request with INVALID_DATA. [RawData: %x]", request);
                invalid_data = true;
            }
            else
            {
                ESP_LOGD(TAG, "Opentherm request with bad parity discarted. [RawData: %x]", request);
                return;
            }
        }

        // override binary
        auto itBinaryOverrideList = override_binary_switch_map.find(requestDataID);
        std::vector<OverrideBinarySwitchInfo *> *pBinaryOverrideList =  itBinaryOverrideList != override_binary_switch_map.end() ? itBinaryOverrideList->second : nullptr;
        if(pBinaryOverrideList != nullptr)
        {
            for(OverrideBinarySwitchInfo *pOverride: *pBinaryOverrideList)
            {
                if((pOverride->valueOnRequest == true) && pOverride->binaryswitch->state && pOverride->valueswitch != nullptr)
                {
                    unsigned short origbitfield = mOT->getUInt(request);
                    bool origvalue = origbitfield & (1<<(pOverride->bit - 1));
                    if(origvalue != pOverride->valueswitch->state)
                    {
                        ESP_LOGD(TAG, "Overriding request bit %d (was %d, overriding to %d)", pOverride->bit, origvalue, pOverride->valueswitch->state);
                    }
                    unsigned short newbitfield = (origbitfield & (0xffff - (1<<(pOverride->bit - 1)))) | (pOverride->valueswitch->state << (pOverride->bit - 1));
                    request = mOT->buildRequest(mOT->getMessageType(request), mOT->getDataID(request), newbitfield);
                }
            }
        }

        // override numeric
        auto itNumericOverrideList = override_numeric_switch_map.find(requestDataID);
        std::vector<OverrideNumericSwitchInfo *> *pNumericOverrideList = itNumericOverrideList != override_numeric_switch_map.end() ? itNumericOverrideList->second : nullptr;
        if(pNumericOverrideList != nullptr)
        {
            for(OverrideNumericSwitchInfo *pOverride: *pNumericOverrideList)
            {
                if((pOverride->valueOnRequest == true) && pOverride->binaryswitch->state && pOverride->valuenumber != nullptr)
                {
                    unsigned short origdata = requestDataValue;
                    unsigned short newdata = convert_to_data(pOverride->valuenumber->state, pOverride->valueType);
                    if(origdata != newdata)
                    {
                        ESP_LOGD(TAG, "Overriding request value (was %d, overriding to %d (%d))", origdata, pOverride->valuenumber->state, newdata);
                    }
                    request = mOT->buildRequest(mOT->getMessageType(request), mOT->getDataID(request), newdata);
                }
            }
        }

        // check validity of modified request
        if(!mOT->isValidRequest(request) && !invalid_data)
        {
            ESP_LOGD(TAG, "Opentherm request is not valid and is discarted.");
            return;
        }

        auto itSensorList = acme_sensor_map.find(mOT->getDataID(request));
        std::vector<AcmeSensorInfo *> *pSensorList = itSensorList != acme_sensor_map.end() ? itSensorList->second : nullptr;
        if(pSensorList != nullptr)
        {
            for(AcmeSensorInfo *pSensorInfo: *pSensorList)
            {
                if (pSensorInfo->valueOnRequest == true)
                {
                    switch(pSensorInfo->valueType)
                    {
                        case 0: // u16
                        {
                            unsigned short value = mOT->getUInt(request);
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 1: // s16
                        {
                            short value = request & 0xffff;
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 2: // f16
                        {
                            float value = mOT->getFloat(request);
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 3: // u8LB
                        {
                            unsigned char value = request & 0x00ff;
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 4: // u8HB
                        {
                            unsigned char value = (request & 0xff00) >> 8;
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 5: // s8LB
                        {
                            signed char value = request & 0x00ff;
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 6: // s8HB
                        {
                            signed char value = (request & 0xff00) >> 8;
                            pSensorInfo->acmeSensor->publish_state(value);
                            break;
                        }
                        case 7: // RESPONSE
                        {
                            pSensorInfo->acmeSensor->publish_state((request >> 28) & 7);
                            break;
                        }
                    }
                }
            }
        }

        // acme binary
        auto itBinarySensorList = acme_binary_sensor_map.find(mOT->getDataID(request));
        std::vector<AcmeBinarySensorInfo *> *pBinarySensorList = itBinarySensorList != acme_binary_sensor_map.end() ? itBinarySensorList->second : nullptr;
        if(pBinarySensorList != nullptr)
        {
            for(AcmeBinarySensorInfo *pBinarySensorInfo: *pBinarySensorList)
            {
                if (pBinarySensorInfo->valueOnRequest == true)
                {
                    bool bitvalue = mOT->getUInt(request) & (1<<(pBinarySensorInfo->bit - 1));
                    pBinarySensorInfo->acmeSensor->publish_state(bitvalue);
                }
            }
        }

        // Send the request
        unsigned long response = mOT->sendRequest(request);

        // process response if valid
        if (response && !sOT->parity(response))
        {
            // override numeric
            auto itNumericOverrideList = override_numeric_switch_map.find(requestDataID);
            std::vector<OverrideNumericSwitchInfo *> *pNumericOverrideList = itNumericOverrideList != override_numeric_switch_map.end() ? itNumericOverrideList->second : nullptr;
            if(pNumericOverrideList != nullptr)
            {
                for(OverrideNumericSwitchInfo *pOverride: *pNumericOverrideList)
                {
                    if((pOverride->valueOnRequest == false) && pOverride->binaryswitch->state && pOverride->valuenumber != nullptr)
                    {
                        ESP_LOGD(TAG, "Opentherm original response [MessageType: %s, DataID: %d, Data: %x]", sOT->messageTypeToString(sOT->getMessageType(response)), sOT->getDataID(response), response&0xffff);
                        unsigned long response_org = response;
                        
                        unsigned short origdata = requestDataValue;
                        unsigned short newdata = convert_to_data(pOverride->valuenumber->state, pOverride->valueType);
                        if(origdata != newdata)
                        {
                            ESP_LOGD(TAG, "Overriding response value (was %d, overriding to %d (%d))", origdata, pOverride->valuenumber->state, newdata);
                        }

                        OpenThermMessageType new_message_type = sOT->getMessageType(response);
                        switch(mOT->getMessageType(request))
                        {
                            case READ_DATA:
                            {
                                new_message_type = READ_ACK;
                                break;
                            }
                            case WRITE_DATA:
                            {
                                new_message_type = WRITE_ACK;
                                break;
                            }
                            default:
                            {
                                break;
                            }
                        }

                        response = sOT->buildResponse(new_message_type, sOT->getDataID(response), newdata);

                        // check validity of modified response
                        if(!sOT->isValidResponse(response))
                        {
                          ESP_LOGD(TAG, "Overriding response failed. Returning original response.");
                          response = response_org;
                        }


                    }
                }
            }

            sOT->sendResponse(response);
            ESP_LOGD(TAG, "Opentherm response [MessageType: %s, DataID: %d, Data: %x]", sOT->messageTypeToString(sOT->getMessageType(response)), sOT->getDataID(response), response&0xffff);

            // only if ACK reponse is received, process the data.
            if(sOT->isValidResponse(response))
            {
                ESP_LOGD(TAG, "Opentherm response valid, processing sensors...");
                
                // acme
                auto itSensorList = acme_sensor_map.find(sOT->getDataID(response));
                std::vector<AcmeSensorInfo *> *pSensorList = itSensorList != acme_sensor_map.end() ? itSensorList->second : nullptr;
                if(pSensorList != nullptr)
                {
                    for(AcmeSensorInfo *pSensorInfo: *pSensorList)
                    {
                        if (pSensorInfo->valueOnRequest == false)
                        {
                            switch(pSensorInfo->valueType)
                            {
                                case 0: // u16
                                {
                                    unsigned short value = sOT->getUInt(response);
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 1: // s16
                                {
                                    short value = response & 0xffff;
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 2: // f16
                                {
                                    float value = sOT->getFloat(response);
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 3: // u8LB
                                {
                                    unsigned char value = response & 0x00ff;
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 4: // u8HB
                                {
                                    unsigned char value = (response & 0xff00) >> 8;
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 5: // s8LB
                                {
                                    signed char value = response & 0x00ff;
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 6: // s8HB
                                {
                                    signed char value = (response & 0xff00) >> 8;
                                    pSensorInfo->acmeSensor->publish_state(value);
                                    break;
                                }
                                case 7: // RESPONSE
                                {
                                    pSensorInfo->acmeSensor->publish_state((response >> 28) & 7);
                                    break;
                                }
                            }
                        }
                    }
                }

                // acme binary
                auto itBinarySensorList = acme_binary_sensor_map.find(sOT->getDataID(response));
                std::vector<AcmeBinarySensorInfo *> *pBinarySensorList = itBinarySensorList != acme_binary_sensor_map.end() ? itBinarySensorList->second : nullptr;
                if(pBinarySensorList != nullptr)
                {
                    for(AcmeBinarySensorInfo *pBinarySensorInfo: *pBinarySensorList)
                    {
                        if (pBinarySensorInfo->valueOnRequest == false)
                        {
                            bool bitvalue = sOT->getUInt(response) & (1<<(pBinarySensorInfo->bit - 1));
                            pBinarySensorInfo->acmeSensor->publish_state(bitvalue);
                        }
                    }
                }
            }
            else
            {
                ESP_LOGD(TAG, "Opentherm response invalid.");
            }
        }
        else
        {
            ESP_LOGD(TAG, "Opentherm - no response or bad parity");
        }
    }


    template<typename T> T OpenthermGW::convert_to_type(double value)
    {
        T retvalue;

        if (value < std::numeric_limits<T>::min())
            retvalue = std::numeric_limits<T>::min();
        else if (value > std::numeric_limits<T>::max())
            retvalue = std::numeric_limits<T>::max();
        else retvalue = static_cast<T>(value);

        return retvalue;
    }

    unsigned short OpenthermGW::convert_to_data(double value, int type)
    {
        unsigned short data = 0;

        switch (type)
        {
            case 0: // u16
            {
                data = convert_to_type<unsigned short>(value);
                break;
            }
            case 1: // s16
            {
                data = convert_to_type<signed short>(value);
                break;
            }
            case 2: // f16
            {
                if (value < -128)
                    value = -128;
                else if (value > 127)
                    value = 127;

                data = (value < 0) ? -(0x10000L - value * 256) : value * 256;
                break;
            }
            case 3: // u8LB
            {
                data = convert_to_type<unsigned char>(value);
                data &= 0x00ff;
                break;
            }
            case 4: // u8HB
            {
                data = convert_to_type<unsigned char>(value) << 8;
                break;
            }
            case 5: // s8LB
            {
                data = convert_to_type<signed char>(value);
                data &= 0x00ff;
                break;
            }
            case 6: // s8HB
            {
                data = convert_to_type<signed char>(value) << 8;
                break;
            }
            default:
            {
                data = 0;
                break;
            }
        }

        return data;
    }


    void OpenthermGW::add_sensor_acme(sensor::Sensor *s, int messageid, bool valueonrequest, int valuetype)
    {
        AcmeSensorInfo *pAcmeSensorInfo = new AcmeSensorInfo();
        pAcmeSensorInfo->messageID = messageid;
        pAcmeSensorInfo->valueOnRequest = valueonrequest;
        pAcmeSensorInfo->valueType = valuetype;
        pAcmeSensorInfo->acmeSensor = s;

        std::vector<AcmeSensorInfo *> *pSensorList = acme_sensor_map[pAcmeSensorInfo->messageID];
        if(pSensorList == nullptr)
        {
            pSensorList = new std::vector<AcmeSensorInfo *>();
            acme_sensor_map[pAcmeSensorInfo->messageID] = pSensorList;
        }
        pSensorList->push_back(pAcmeSensorInfo);
    }

    void OpenthermGW::add_sensor_acme_binary(binary_sensor::BinarySensor *s, int messageid, bool valueonrequest, int bit)
    {
        AcmeBinarySensorInfo *pAcmeBinarySensorInfo = new AcmeBinarySensorInfo();
        pAcmeBinarySensorInfo->messageID = messageid;
        pAcmeBinarySensorInfo->valueOnRequest = valueonrequest;
        pAcmeBinarySensorInfo->bit = bit;
        pAcmeBinarySensorInfo->acmeSensor = s;

        std::vector<AcmeBinarySensorInfo *> *pSensorList = acme_binary_sensor_map[pAcmeBinarySensorInfo->messageID];
        if(pSensorList == nullptr)
        {
            pSensorList = new std::vector<AcmeBinarySensorInfo *>();
            acme_binary_sensor_map[pAcmeBinarySensorInfo->messageID] = pSensorList;
        }
        pSensorList->push_back(pAcmeBinarySensorInfo);
    }

    void OpenthermGW::add_override_switch(openthermgw::OverrideBinarySwitch *s, int messageid, bool valueonrequest, int bit, openthermgw::SimpleSwitch *v)
    {
        OverrideBinarySwitchInfo *pOverrideBinarySwitchInfo = new OverrideBinarySwitchInfo();
        pOverrideBinarySwitchInfo->messageID = messageid;
        pOverrideBinarySwitchInfo->valueOnRequest = valueonrequest;
        pOverrideBinarySwitchInfo->bit = bit;
        pOverrideBinarySwitchInfo->binaryswitch = s;
        pOverrideBinarySwitchInfo->valueswitch = v;

        std::vector<OverrideBinarySwitchInfo *> *pSwitchList = override_binary_switch_map[pOverrideBinarySwitchInfo->messageID];
        if(pSwitchList == nullptr)
        {
            pSwitchList = new std::vector<OverrideBinarySwitchInfo *>();
            override_binary_switch_map[pOverrideBinarySwitchInfo->messageID] = pSwitchList;
        }
        pSwitchList->push_back(pOverrideBinarySwitchInfo);
    }

    void OpenthermGW::add_override_numeric_switch(openthermgw::OverrideBinarySwitch *s, int messageid, bool valueonrequest, int valuetype, openthermgw::SimpleNumber *v)
    {
        OverrideNumericSwitchInfo *pOverrideNumericSwitchInfo = new OverrideNumericSwitchInfo();
        pOverrideNumericSwitchInfo->messageID = messageid;
        pOverrideNumericSwitchInfo->valueOnRequest = valueonrequest;
        pOverrideNumericSwitchInfo->valueType = valuetype;
        pOverrideNumericSwitchInfo->binaryswitch = s;
        pOverrideNumericSwitchInfo->valuenumber = v;

        std::vector<OverrideNumericSwitchInfo *> *pSwitchList = override_numeric_switch_map[pOverrideNumericSwitchInfo->messageID];
        if(pSwitchList == nullptr)
        {
            pSwitchList = new std::vector<OverrideNumericSwitchInfo *>();
            override_numeric_switch_map[pOverrideNumericSwitchInfo->messageID] = pSwitchList;
        }
        pSwitchList->push_back(pOverrideNumericSwitchInfo);
    }

    void OpenthermGW::setup()
    {
        // This will be called once to set up the component
        // think of it as the setup() call in Arduino
        ESP_LOGD(TAG, "Setup");

        master_in_pin_sensor->publish_state(master_in_pin_);
        master_out_pin_sensor->publish_state(master_out_pin_);
        slave_in_pin_sensor->publish_state(slave_in_pin_);
        slave_out_pin_sensor->publish_state(slave_out_pin_);

        mOT = new OpenTherm(slave_in_pin_, slave_out_pin_); // master OT is paired with SLAVE pins (thermostat)
        sOT = new OpenTherm(master_in_pin_, master_out_pin_, true);

        mOT->begin(mHandleInterrupt);
        sOT->begin(sHandleInterrupt, processRequest);
    }

    void OpenthermGW::update()
    {
        ESP_LOGD(TAG, "acme messages handdled: %d", acme_sensor_map.size());
        ESP_LOGD(TAG, "acme binary messages handled: %d", acme_binary_sensor_map.size());
        ESP_LOGD(TAG, "acme binary overrides handled: %d", override_binary_switch_map.size());
        ESP_LOGD(TAG, "acme numeric overrides handled: %d", override_numeric_switch_map.size());
    }

    void OpenthermGW::loop()
    {
        sOT->process();
    }


//////////////////////////////////////////////////////

    OverrideBinarySwitch::OverrideBinarySwitch()
    {
    }

    void OverrideBinarySwitch::setup()
    {
        this->state = this->get_initial_state_with_restore_mode().value_or(false);
    }

    void OverrideBinarySwitch::write_state(bool state)
    {
        if(state_ != state)
        {
            state_ = state;
            this->publish_state(state);
        }
    }

//////////////////////////////////////////////////////

    SimpleSwitch::SimpleSwitch()
    {
    }

    void SimpleSwitch::setup()
    {
        this->state = this->get_initial_state_with_restore_mode().value_or(false);
    }

    void SimpleSwitch::write_state(bool s)
    {
        this->publish_state(s);
    }

//////////////////////////////////////////////////////

    void SimpleNumber::setup()
    {
        float value;
        if (!this->restore_value_)
        {
            value = this->initial_value_;
        }
        else
        {
            this->pref_ = global_preferences->make_preference<float>(this->get_object_id_hash());
            if (!this->pref_.load(&value))
            {
                if (!std::isnan(this->initial_value_))
                {
                    value = this->initial_value_;
                }
                else
                {
                    value = this->traits.get_min_value();
                }
            }
        }
        this->publish_state(value);
    }

    void SimpleNumber::control(float value)
    {
        this->set_trigger_->trigger(value);

        this->publish_state(value);

        if (this->restore_value_)
            this->pref_.save(&value);
    }

    void SimpleNumber::dump_config() { LOG_NUMBER("", "Simple Number", this); }
}
}
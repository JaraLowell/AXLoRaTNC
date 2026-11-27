#pragma once

#if !defined(AXLORA_VARIANT_DEVKITV1_E22) && !defined(AXLORA_VARIANT_HELTEC_V3) && !defined(AXLORA_VARIANT_TBEAM) && !defined(AXLORA_VARIANT_LILYGO_T3_V161) && !defined(AXLORA_VARIANT_TBEAM_SUPREME_433) && !defined(AXLORA_VARIANT_STATION_G2) && !defined(AXLORA_VARIANT_SENSECAP_SOLAR)
#error "Select an AXLoRa hardware variant with a build flag, for example -DAXLORA_VARIANT_DEVKITV1_E22"
#endif

#if defined(AXLORA_VARIANT_SENSECAP_SOLAR)
#include "axlora_variant.h"
#else
#include "variant.h"
#endif

namespace axlora {

static constexpr uint16_t MAX_PACKET_BYTES = 330;
static constexpr uint16_t MAX_LORA_PACKET_BYTES = 255;

}

# SPDX-License-Identifier: Apache-2.0

set(ZEPHYR_CURRENT_LIBRARY loramac-node)

zephyr_library_sources_ifdef(CONFIG_LORA_SX1276 sx1276.c)

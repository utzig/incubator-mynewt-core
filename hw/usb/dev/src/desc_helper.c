/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <usb/usb.h>
#include <dev/class.h>

#include <stdlib.h>
#include <string.h>

static usb_language_list_t *g_lang_list = NULL;
static int g_string_count = 0;

/*
 * Allocates a string descriptor array from a textual string.
 */
static uint8_t *
_desc_for_str_alloc(char *str)
{
    int i;
    size_t len;
    uint8_t *s;

    if (!str) {
        return NULL;
    }

    len = strlen(str);
    if (len < 2 || len > 253) {
        return NULL;
    }

    s = malloc(2 * len + 2);
    if (!s) {
        return NULL;
    }

    s[0] = 2 * len + 2;
    s[1] = DESC_TYPE_STRING;
    for (i = 0; i < len; i++) {
        s[i*2 + 2] = str[i];
        s[i*2 + 3] = 0;
    }

    return s;
}

int
usb_desc_strings_alloc(void)
{
    usb_language_t *langs = NULL;
    uint16_t lang_id = MYNEWT_VAL(USB_DESC_STRING0);
    uint8_t *u0 = NULL;
    uint8_t *u1 = NULL;
    uint8_t *u2 = NULL;
    uint8_t *u3 = NULL;
    uint8_t *u4 = NULL;
    uint8_t *u5 = NULL;
    char *string1 = MYNEWT_VAL_USB_DESC_STRING1;
    char *string2 = MYNEWT_VAL_USB_DESC_STRING2;
    char *string3 = MYNEWT_VAL_USB_DESC_STRING3;
    char *string4 = MYNEWT_VAL_USB_DESC_STRING4;
    char *string5 = MYNEWT_VAL_USB_DESC_STRING5;

    g_lang_list = malloc(sizeof(usb_language_list_t));
    if (!g_lang_list) {
        return USB_ERR;
    }

    g_lang_list->languageString = NULL;
    g_lang_list->stringLength   = 0;
    g_lang_list->languageList   = NULL;
    g_lang_list->count          = 1;

#if !MYNEWT_VAL(USB_DESC_STRING0)
    return 0;
#endif

    /* TODO: support more than one language? */
    langs = malloc(sizeof(usb_language_t));
    if (!langs) {
        return USB_ERR;
    }

    langs[0].string = NULL;
    langs[0].length = 0;
    langs[0].languageId = lang_id;

    u0 = malloc(4);
    if (!u0) {
        return USB_ERR;
    }

    /* String0 is the language ID, fill in manually */
    u0[0] = 4;
    u0[1] = DESC_TYPE_STRING;
    u0[2] = lang_id;
    u0[3] = lang_id >> 8;
    g_string_count++;

    u1 = _desc_for_str_alloc(string1);
    if (!u1) goto done;
    g_string_count++;

    u2 = _desc_for_str_alloc(string2);
    if (!u2) goto done;
    g_string_count++;

    u3 = _desc_for_str_alloc(string3);
    if (!u3) goto done;
    g_string_count++;

    u4 = _desc_for_str_alloc(string4);
    if (!u2) goto done;
    g_string_count++;

    u5 = _desc_for_str_alloc(string5);
    if (!u5) goto done;
    g_string_count++;

done:
    langs[0].string = malloc(g_string_count * sizeof(uint8_t *));
    if (!langs[0].string) {
        return USB_ERR;
    }

    langs[0].length = malloc(g_string_count * sizeof(uint32_t));
    if (!langs[0].length) {
        return USB_ERR;
    }

    if (u0) {
        langs[0].string[0] = u0;
        langs[0].length[0] = u0[0];
    }
    if (u1) {
        langs[0].string[1] = u1;
        langs[0].length[1] = u1[0];
    }
    if (u2) {
        langs[0].string[2] = u2;
        langs[0].length[2] = u2[0];
    }
    if (u3) {
        langs[0].string[3] = u3;
        langs[0].length[3] = u3[0];
    }
    if (u4) {
        langs[0].string[4] = u4;
        langs[0].length[4] = u4[0];
    }
    if (u5) {
        langs[0].string[5] = u5;
        langs[0].length[5] = u5[0];
    }

    g_lang_list->languageString = u0;
    g_lang_list->stringLength   = u0[0];
    g_lang_list->languageList   = langs;

    return 0;
}

int
usb_desc_get_string(usb_device_handle handle, usb_desc_string_t *desc)
{
    uint8_t langid = 0;
    uint8_t langidx = g_string_count;

    if (!g_lang_list) {
        return USB_ERR;
    }

    if (desc->str_idx == 0) {
        desc->buf = (uint8_t *)g_lang_list->languageString;
        desc->len = g_lang_list->stringLength;
    } else {
        /* TODO: support more languages? */
        for (; langid < 1; langid++) {
            if (desc->lang_id == g_lang_list->languageList[langid].languageId) {
                if (desc->str_idx < g_string_count) {
                    langidx = desc->str_idx;
                }
                break;
            }
        }

        if (langidx == g_string_count) {
            return USB_INVALID_REQ;
        }
        desc->buf = g_lang_list->languageList[langid].string[langidx];
        desc->len = g_lang_list->languageList[langid].length[langidx];
    }

    return 0;
}

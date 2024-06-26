// Copyright 2017-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <jsmn-changed.h>
#include <json_parser.h>

#include "app_defs.h"

#ifdef CTN_ASK_PARSER
#include <esp_log.h>

static const char *TAG = "[json_components]";
#endif

#if defined(BLYNK_APPS)
bool vent_power_on = false;
uint8_t vent_step = 0;
#endif

static bool token_matches_str(jparse_ctx_t *ctx, json_tok_t *tok, char *str)
{
    char *js = ctx->js;
    return ((strncmp(js + tok->start, str, strlen(str)) == 0)
            && (strlen(str) == (size_t) (tok->end - tok->start)));
}

static json_tok_t *json_skip_elem(json_tok_t *token)
{
    json_tok_t *cur = token;
    int cnt = cur->size;
    while (cnt--) {
        cur++;
        cur = json_skip_elem(cur);
    }
    return cur;
}

static int json_tok_to_bool(jparse_ctx_t *jctx, json_tok_t *tok, bool *val)
{
    if (token_matches_str(jctx, tok, "true") || token_matches_str(jctx, tok, "1")) {
        *val = true;
    } else if  (token_matches_str(jctx, tok, "false") || token_matches_str(jctx, tok, "0")) {
        *val = false;
    } else {
        return -OS_FAIL;
    }
    return OS_SUCCESS;
}

static int json_tok_to_int(jparse_ctx_t *jctx, json_tok_t *tok, int *val)
{
    char *tok_start = &jctx->js[tok->start];
    char *tok_end = &jctx->js[tok->end];
    char *endptr;
    int i = strtoul(tok_start, &endptr, 10);
    if (endptr == tok_end) {
        *val = i;
        return OS_SUCCESS;
    }
    return -OS_FAIL;
}

static int json_tok_to_int64(jparse_ctx_t *jctx, json_tok_t *tok, int64_t *val)
{
    char *tok_start = &jctx->js[tok->start];
    char *tok_end = &jctx->js[tok->end];
    char *endptr;
    int64_t i64 = strtoull(tok_start, &endptr, 10);
    if (endptr == tok_end) {
        *val = i64;
        return OS_SUCCESS;
    }
    return -OS_FAIL;
}

static int json_tok_to_float(jparse_ctx_t *jctx, json_tok_t *tok, float *val)
{
    char *tok_start = &jctx->js[tok->start];
    char *tok_end = &jctx->js[tok->end];
    char *endptr;
    float f = strtof(tok_start, &endptr);
    if (endptr == tok_end) {
        *val = f;
        return OS_SUCCESS;
    }
    return -OS_FAIL;
}

static int json_tok_to_string(jparse_ctx_t *jctx, json_tok_t *tok, char *val, int size)
{
#ifdef CTN_ASK_PARSER
	char *power_off		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.a7241fa8-11b5-49df-902e-baced357d257";
	char *power_on		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.b33fccff-ecc2-4e3d-bddf-4608e97b6520";
	char *step_one		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.ba975a23-1a94-4e9b-a63f-d2f96ee6b4fd";
	char *step_two		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.c9f80039-1984-41ee-ade7-1f642c90d42a";
	char *step_three	= "cid:ValidatedSpeakDirective_amzn1.ask.skill.cd696c5b-aeb7-41df-a2ec-2e3400d5a662";
	char *one_hour		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.1432836d-53a0-4af5-96ab-dc21ae126831";
	char *four_hours	= "cid:ValidatedSpeakDirective_amzn1.ask.skill.02407480-15e2-4b1f-a663-3d7d835c5b50";
	char *eight_hours	= "cid:ValidatedSpeakDirective_amzn1.ask.skill.a7e34dcd-8204-4265-857e-1dd64e7a1892";
	char *ai_mode		= "cid:ValidatedSpeakDirective_amzn1.ask.skill.ebf1f4d7-5407-4705-a2b2-e13eb0b1b1c6";
#endif

    if ((tok->end - tok->start) > (size - 1)) {
        return -OS_FAIL;
    }
    strncpy(val, jctx->js + tok->start, tok->end - tok->start);

#ifdef CTN_ASK_PARSER
	//ESP_LOGE(TAG, "len=[%d] string=[%s]\n", (tok->end - tok->start), val);
	if(strstr(val, power_off))
	{
		ESP_LOGE(TAG, "power_off, to do...\n");
#if defined(BLYNK_APPS)
		vent_power_on = false;
#endif
	}
	else if(strstr(val, power_on))
	{
		ESP_LOGE(TAG, "power_on, to do...\n");
#if defined(BLYNK_APPS)
		vent_power_on = true;
#endif
	}
	else if(strstr(val, step_one))
	{
		ESP_LOGE(TAG, "step_one, to do...\n");
#if defined(BLYNK_APPS)
		vent_step = 11;
#endif
	}
	else if(strstr(val, step_two))
	{
		ESP_LOGE(TAG, "step_two, to do...\n");
#if defined(BLYNK_APPS)
		vent_step = 12;
#endif
	}
	else if(strstr(val, step_three))
	{
		ESP_LOGE(TAG, "step_three, to do...\n");
#if defined(BLYNK_APPS)
		vent_step = 13;
#endif
	}
	else if(strstr(val, one_hour))
	{
		ESP_LOGE(TAG, "one_hour, to do...\n");
	}
	else if(strstr(val, four_hours))
	{
		ESP_LOGE(TAG, "four_hours, to do...\n");
	}
	else if(strstr(val, eight_hours))
	{
		ESP_LOGE(TAG, "eight_hours, to do...\n");
	}
	else if(strstr(val, ai_mode))
	{
		ESP_LOGE(TAG, "ai_mode, to do...\n");
	}
#endif

    val[tok->end - tok->start] = 0;
    return OS_SUCCESS;
}

static json_tok_t *json_obj_search(jparse_ctx_t *jctx, char *key)
{
    json_tok_t *tok = jctx->cur;
    int size = tok->size;
    if (size <= 0) {
        return NULL;
    }
    if (tok->type != JSMN_OBJECT) {
        return NULL;
    }

    while (size--) {
        tok++;
        if (token_matches_str(jctx, tok, key)) {
            return tok;
        }
        tok = json_skip_elem(tok);
    }
    return NULL;
}

static json_tok_t *json_obj_get_val_tok(jparse_ctx_t *jctx, char *name, _jsmntype_t type)
{
    json_tok_t *tok = json_obj_search(jctx, name);
    if (!tok) {
        return NULL;
    }
    tok++;
    if (tok->type != type) {
        return NULL;
    }
    return tok;
}

int json_obj_get_array(jparse_ctx_t *jctx, char *name, int *num_elem)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_ARRAY);
    if (!tok) {
        return -OS_FAIL;
    }
    jctx->cur = tok;
    *num_elem = tok->size;
    return OS_SUCCESS;
}

int json_obj_leave_array(jparse_ctx_t *jctx)
{
    /* The array's parent will be the key */
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];

    /* The key's parent will be the actual parent object */
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];
    return OS_SUCCESS;
}

int json_obj_get_object(jparse_ctx_t *jctx, char *name)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_OBJECT);
    if (!tok) {
        return -OS_FAIL;
    }
    jctx->cur = tok;
    return OS_SUCCESS;
}

int json_obj_leave_object(jparse_ctx_t *jctx)
{
    /* The objects's parent will be the key */
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];

    /* The key's parent will be the actual parent object */
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];
    return OS_SUCCESS;
}

int json_obj_get_bool(jparse_ctx_t *jctx, char *name, bool *val)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_bool(jctx, tok, val);
}

int json_obj_get_int(jparse_ctx_t *jctx, char *name, int *val)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_int(jctx, tok, val);
}

int json_obj_get_int64(jparse_ctx_t *jctx, char *name, int64_t *val)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_int64(jctx, tok, val);
}

int json_obj_get_float(jparse_ctx_t *jctx, char *name, float *val)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_float(jctx, tok, val);
}

int json_obj_get_string(jparse_ctx_t *jctx, char *name, char *val, int size)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_STRING);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_string(jctx, tok, val, size);
}

int json_obj_get_strlen(jparse_ctx_t *jctx, char *name, int *strlen)
{
    json_tok_t *tok = json_obj_get_val_tok(jctx, name, JSMN_STRING);
    if (!tok) {
        return -OS_FAIL;
    }
    *strlen = tok->end - tok->start;
    return OS_SUCCESS;
}

static json_tok_t *json_arr_search(jparse_ctx_t *ctx, uint32_t index)
{
    json_tok_t *tok = ctx->cur;
    if ((tok->type != JSMN_ARRAY) || (tok->size <= 0)) {
        return NULL;
    }
    if (index > (uint32_t)(tok->size - 1)) {
        return NULL;
    }
    /* Increment by 1, so that token points to index 0 */
    tok++;
    while (index--) {
        tok = json_skip_elem(tok);
        tok++;
    }
    return tok;
}
static json_tok_t *json_arr_get_val_tok(jparse_ctx_t *jctx, uint32_t index, _jsmntype_t type)
{
    json_tok_t *tok = json_arr_search(jctx, index);
    if (!tok) {
        return NULL;
    }
    if (tok->type != type) {
        return NULL;
    }
    return tok;
}

int json_arr_get_array(jparse_ctx_t *jctx, uint32_t index)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_ARRAY);
    if (!tok) {
        return -OS_FAIL;
    }
    jctx->cur = tok;
    return OS_SUCCESS;
}

int json_arr_leave_array(jparse_ctx_t *jctx)
{
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];
    return OS_SUCCESS;
}

int json_arr_get_object(jparse_ctx_t *jctx, uint32_t index)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_OBJECT);
    if (!tok) {
        return -OS_FAIL;
    }
    jctx->cur = tok;
    return OS_SUCCESS;
}

int json_arr_leave_object(jparse_ctx_t *jctx)
{
    if (jctx->cur->parent < 0) {
        return -OS_FAIL;
    }
    jctx->cur = &jctx->tokens[jctx->cur->parent];
    return OS_SUCCESS;
}

int json_arr_get_bool(jparse_ctx_t *jctx, uint32_t index, bool *val)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_bool(jctx, tok, val);
}

int json_arr_get_int(jparse_ctx_t *jctx, uint32_t index, int *val)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_int(jctx, tok, val);
}

int json_arr_get_int64(jparse_ctx_t *jctx, uint32_t index, int64_t *val)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_int64(jctx, tok, val);
}

int json_arr_get_float(jparse_ctx_t *jctx, uint32_t index, float *val)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_PRIMITIVE);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_float(jctx, tok, val);
}

int json_arr_get_string(jparse_ctx_t *jctx, uint32_t index, char *val, int size)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_STRING);
    if (!tok) {
        return -OS_FAIL;
    }
    return json_tok_to_string(jctx, tok, val, size);
}

int json_arr_get_strlen(jparse_ctx_t *jctx, uint32_t index, int *strlen)
{
    json_tok_t *tok = json_arr_get_val_tok(jctx, index, JSMN_STRING);
    if (!tok) {
        return -OS_FAIL;
    }
    *strlen = tok->end - tok->start;
    return OS_SUCCESS;
}

int json_parse_start(jparse_ctx_t *jctx, char *js, int len)
{
    memset(jctx, 0, sizeof(jparse_ctx_t));
    __jsmn_init(&jctx->parser);
    int num_tokens = __jsmn_parse(&jctx->parser, js, len, NULL, 0);
    if (num_tokens <= 0) {
        return -OS_FAIL;
    }
    jctx->num_tokens = num_tokens;
    jctx->tokens = calloc(num_tokens, sizeof(json_tok_t));
    if (!jctx->tokens) {
        return -OS_FAIL;
    }
    jctx->js = js;
    __jsmn_init(&jctx->parser);
    int ret = __jsmn_parse(&jctx->parser, js, len, jctx->tokens, jctx->num_tokens);
    if (ret <= 0) {
        free(jctx->tokens);
        memset(jctx, 0, sizeof(jparse_ctx_t));
        return -OS_FAIL;
    }
    jctx->cur = jctx->tokens;
    return OS_SUCCESS;
}

int json_parse_end(jparse_ctx_t *jctx)
{
    if (jctx->tokens) {
        free(jctx->tokens);
    }
    memset(jctx, 0, sizeof(jparse_ctx_t));
    return OS_SUCCESS;
}

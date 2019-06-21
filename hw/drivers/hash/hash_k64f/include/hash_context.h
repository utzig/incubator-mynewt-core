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

#ifndef __HASH_CONTEXT_H__
#define __HASH_CONTEXT_H__

#include "hash/hash.h"

/*
 * The following HASH algos are not implemented or not
 * supported on K64F (SHA-224 is not supported)
 */

struct hash_md5_context {
    void *dev;
};

struct hash_sha1_context {
    void *dev;
};

struct hash_sha224_context {
    void *dev;
};

/*
 * SHA-256 is implemented.
 */

struct hash_sha256_context {
    /* Store device pointer */
    void *dev;
    /* Total length of the hashed data */
    uint32_t len;
    /* Current context output of the hash */
    unsigned int output[SHA256_DIGEST_LEN >> 2];
    /* Last block added, used to pad on finish() */
    uint8_t pad[SHA256_BLOCK_LEN];
    /* Amount added to pad, used on finish() */
    uint8_t remain;
};

#endif /* __HASH_CONTEXT_H__ */

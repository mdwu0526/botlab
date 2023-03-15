// THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
// BY HAND!!
//
// Generated by lcm-gen

#include <string.h>
#include "mbot_lcm_msgs_joystick_t.h"

static int __mbot_lcm_msgs_joystick_t_hash_computed;
static uint64_t __mbot_lcm_msgs_joystick_t_hash;

uint64_t __mbot_lcm_msgs_joystick_t_hash_recursive(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for (fp = p; fp != NULL; fp = fp->parent)
        if (fp->v == __mbot_lcm_msgs_joystick_t_get_hash)
            return 0;

    __lcm_hash_ptr cp;
    cp.parent =  p;
    cp.v = __mbot_lcm_msgs_joystick_t_get_hash;
    (void) cp;

    uint64_t hash = (uint64_t)0x411d5cfa5f4fa383LL
         + __int64_t_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __float_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
         + __int8_t_hash_recursive(&cp)
        ;

    return (hash<<1) + ((hash>>63)&1);
}

int64_t __mbot_lcm_msgs_joystick_t_get_hash(void)
{
    if (!__mbot_lcm_msgs_joystick_t_hash_computed) {
        __mbot_lcm_msgs_joystick_t_hash = (int64_t)__mbot_lcm_msgs_joystick_t_hash_recursive(NULL);
        __mbot_lcm_msgs_joystick_t_hash_computed = 1;
    }

    return __mbot_lcm_msgs_joystick_t_hash;
}

int __mbot_lcm_msgs_joystick_t_encode_array(void *buf, int offset, int maxlen, const mbot_lcm_msgs_joystick_t *p, int elements)
{
    int pos = 0, element;
    int thislen;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].left_analog_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].left_analog_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].right_analog_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].right_analog_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].right_trigger), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].left_trigger), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].dpad_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &(p[element].dpad_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_A), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_B), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_5), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_l1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_r1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_l2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_r2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_select), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_start), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_12), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_left_analog), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_right_analog), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &(p[element].button_15), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int mbot_lcm_msgs_joystick_t_encode(void *buf, int offset, int maxlen, const mbot_lcm_msgs_joystick_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __mbot_lcm_msgs_joystick_t_get_hash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    thislen = __mbot_lcm_msgs_joystick_t_encode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int __mbot_lcm_msgs_joystick_t_encoded_array_size(const mbot_lcm_msgs_joystick_t *p, int elements)
{
    int size = 0, element;
    for (element = 0; element < elements; element++) {

        size += __int64_t_encoded_array_size(&(p[element].timestamp), 1);

        size += __float_encoded_array_size(&(p[element].left_analog_X), 1);

        size += __float_encoded_array_size(&(p[element].left_analog_Y), 1);

        size += __float_encoded_array_size(&(p[element].right_analog_X), 1);

        size += __float_encoded_array_size(&(p[element].right_analog_Y), 1);

        size += __float_encoded_array_size(&(p[element].right_trigger), 1);

        size += __float_encoded_array_size(&(p[element].left_trigger), 1);

        size += __float_encoded_array_size(&(p[element].dpad_X), 1);

        size += __float_encoded_array_size(&(p[element].dpad_Y), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_A), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_B), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_2), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_X), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_Y), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_5), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_l1), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_r1), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_l2), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_r2), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_select), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_start), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_12), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_left_analog), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_right_analog), 1);

        size += __int8_t_encoded_array_size(&(p[element].button_15), 1);

    }
    return size;
}

int mbot_lcm_msgs_joystick_t_encoded_size(const mbot_lcm_msgs_joystick_t *p)
{
    return 8 + __mbot_lcm_msgs_joystick_t_encoded_array_size(p, 1);
}

int __mbot_lcm_msgs_joystick_t_decode_array(const void *buf, int offset, int maxlen, mbot_lcm_msgs_joystick_t *p, int elements)
{
    int pos = 0, thislen, element;

    for (element = 0; element < elements; element++) {

        thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].timestamp), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].left_analog_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].left_analog_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].right_analog_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].right_analog_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].right_trigger), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].left_trigger), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].dpad_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &(p[element].dpad_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_A), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_B), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_X), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_Y), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_5), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_l1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_r1), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_l2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_r2), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_select), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_start), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_12), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_left_analog), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_right_analog), 1);
        if (thislen < 0) return thislen; else pos += thislen;

        thislen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &(p[element].button_15), 1);
        if (thislen < 0) return thislen; else pos += thislen;

    }
    return pos;
}

int __mbot_lcm_msgs_joystick_t_decode_array_cleanup(mbot_lcm_msgs_joystick_t *p, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_decode_array_cleanup(&(p[element].timestamp), 1);

        __float_decode_array_cleanup(&(p[element].left_analog_X), 1);

        __float_decode_array_cleanup(&(p[element].left_analog_Y), 1);

        __float_decode_array_cleanup(&(p[element].right_analog_X), 1);

        __float_decode_array_cleanup(&(p[element].right_analog_Y), 1);

        __float_decode_array_cleanup(&(p[element].right_trigger), 1);

        __float_decode_array_cleanup(&(p[element].left_trigger), 1);

        __float_decode_array_cleanup(&(p[element].dpad_X), 1);

        __float_decode_array_cleanup(&(p[element].dpad_Y), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_A), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_B), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_2), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_X), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_Y), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_5), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_l1), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_r1), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_l2), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_r2), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_select), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_start), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_12), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_left_analog), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_right_analog), 1);

        __int8_t_decode_array_cleanup(&(p[element].button_15), 1);

    }
    return 0;
}

int mbot_lcm_msgs_joystick_t_decode(const void *buf, int offset, int maxlen, mbot_lcm_msgs_joystick_t *p)
{
    int pos = 0, thislen;
    int64_t hash = __mbot_lcm_msgs_joystick_t_get_hash();

    int64_t this_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (this_hash != hash) return -1;

    thislen = __mbot_lcm_msgs_joystick_t_decode_array(buf, offset + pos, maxlen - pos, p, 1);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int mbot_lcm_msgs_joystick_t_decode_cleanup(mbot_lcm_msgs_joystick_t *p)
{
    return __mbot_lcm_msgs_joystick_t_decode_array_cleanup(p, 1);
}

int __mbot_lcm_msgs_joystick_t_clone_array(const mbot_lcm_msgs_joystick_t *p, mbot_lcm_msgs_joystick_t *q, int elements)
{
    int element;
    for (element = 0; element < elements; element++) {

        __int64_t_clone_array(&(p[element].timestamp), &(q[element].timestamp), 1);

        __float_clone_array(&(p[element].left_analog_X), &(q[element].left_analog_X), 1);

        __float_clone_array(&(p[element].left_analog_Y), &(q[element].left_analog_Y), 1);

        __float_clone_array(&(p[element].right_analog_X), &(q[element].right_analog_X), 1);

        __float_clone_array(&(p[element].right_analog_Y), &(q[element].right_analog_Y), 1);

        __float_clone_array(&(p[element].right_trigger), &(q[element].right_trigger), 1);

        __float_clone_array(&(p[element].left_trigger), &(q[element].left_trigger), 1);

        __float_clone_array(&(p[element].dpad_X), &(q[element].dpad_X), 1);

        __float_clone_array(&(p[element].dpad_Y), &(q[element].dpad_Y), 1);

        __int8_t_clone_array(&(p[element].button_A), &(q[element].button_A), 1);

        __int8_t_clone_array(&(p[element].button_B), &(q[element].button_B), 1);

        __int8_t_clone_array(&(p[element].button_2), &(q[element].button_2), 1);

        __int8_t_clone_array(&(p[element].button_X), &(q[element].button_X), 1);

        __int8_t_clone_array(&(p[element].button_Y), &(q[element].button_Y), 1);

        __int8_t_clone_array(&(p[element].button_5), &(q[element].button_5), 1);

        __int8_t_clone_array(&(p[element].button_l1), &(q[element].button_l1), 1);

        __int8_t_clone_array(&(p[element].button_r1), &(q[element].button_r1), 1);

        __int8_t_clone_array(&(p[element].button_l2), &(q[element].button_l2), 1);

        __int8_t_clone_array(&(p[element].button_r2), &(q[element].button_r2), 1);

        __int8_t_clone_array(&(p[element].button_select), &(q[element].button_select), 1);

        __int8_t_clone_array(&(p[element].button_start), &(q[element].button_start), 1);

        __int8_t_clone_array(&(p[element].button_12), &(q[element].button_12), 1);

        __int8_t_clone_array(&(p[element].button_left_analog), &(q[element].button_left_analog), 1);

        __int8_t_clone_array(&(p[element].button_right_analog), &(q[element].button_right_analog), 1);

        __int8_t_clone_array(&(p[element].button_15), &(q[element].button_15), 1);

    }
    return 0;
}

mbot_lcm_msgs_joystick_t *mbot_lcm_msgs_joystick_t_copy(const mbot_lcm_msgs_joystick_t *p)
{
    mbot_lcm_msgs_joystick_t *q = (mbot_lcm_msgs_joystick_t*) malloc(sizeof(mbot_lcm_msgs_joystick_t));
    __mbot_lcm_msgs_joystick_t_clone_array(p, q, 1);
    return q;
}

void mbot_lcm_msgs_joystick_t_destroy(mbot_lcm_msgs_joystick_t *p)
{
    __mbot_lcm_msgs_joystick_t_decode_array_cleanup(p, 1);
    free(p);
}

int mbot_lcm_msgs_joystick_t_publish(lcm_t *lc, const char *channel, const mbot_lcm_msgs_joystick_t *p)
{
      int max_data_size = mbot_lcm_msgs_joystick_t_encoded_size (p);
      uint8_t *buf = (uint8_t*) malloc (max_data_size);
      if (!buf) return -1;
      int data_size = mbot_lcm_msgs_joystick_t_encode (buf, 0, max_data_size, p);
      if (data_size < 0) {
          free (buf);
          return data_size;
      }
      int status = lcm_publish (lc, channel, buf, data_size);
      free (buf);
      return status;
}

struct _mbot_lcm_msgs_joystick_t_subscription_t {
    mbot_lcm_msgs_joystick_t_handler_t user_handler;
    void *userdata;
    lcm_subscription_t *lc_h;
};
static
void mbot_lcm_msgs_joystick_t_handler_stub (const lcm_recv_buf_t *rbuf,
                            const char *channel, void *userdata)
{
    int status;
    mbot_lcm_msgs_joystick_t p;
    memset(&p, 0, sizeof(mbot_lcm_msgs_joystick_t));
    status = mbot_lcm_msgs_joystick_t_decode (rbuf->data, 0, rbuf->data_size, &p);
    if (status < 0) {
        fprintf (stderr, "error %d decoding mbot_lcm_msgs_joystick_t!!!\n", status);
        return;
    }

    mbot_lcm_msgs_joystick_t_subscription_t *h = (mbot_lcm_msgs_joystick_t_subscription_t*) userdata;
    h->user_handler (rbuf, channel, &p, h->userdata);

    mbot_lcm_msgs_joystick_t_decode_cleanup (&p);
}

mbot_lcm_msgs_joystick_t_subscription_t* mbot_lcm_msgs_joystick_t_subscribe (lcm_t *lcm,
                    const char *channel,
                    mbot_lcm_msgs_joystick_t_handler_t f, void *userdata)
{
    mbot_lcm_msgs_joystick_t_subscription_t *n = (mbot_lcm_msgs_joystick_t_subscription_t*)
                       malloc(sizeof(mbot_lcm_msgs_joystick_t_subscription_t));
    n->user_handler = f;
    n->userdata = userdata;
    n->lc_h = lcm_subscribe (lcm, channel,
                                 mbot_lcm_msgs_joystick_t_handler_stub, n);
    if (n->lc_h == NULL) {
        fprintf (stderr,"couldn't reg mbot_lcm_msgs_joystick_t LCM handler!\n");
        free (n);
        return NULL;
    }
    return n;
}

int mbot_lcm_msgs_joystick_t_subscription_set_queue_capacity (mbot_lcm_msgs_joystick_t_subscription_t* subs,
                              int num_messages)
{
    return lcm_subscription_set_queue_capacity (subs->lc_h, num_messages);
}

int mbot_lcm_msgs_joystick_t_unsubscribe(lcm_t *lcm, mbot_lcm_msgs_joystick_t_subscription_t* hid)
{
    int status = lcm_unsubscribe (lcm, hid->lc_h);
    if (0 != status) {
        fprintf(stderr,
           "couldn't unsubscribe mbot_lcm_msgs_joystick_t_handler %p!\n", hid);
        return -1;
    }
    free (hid);
    return 0;
}


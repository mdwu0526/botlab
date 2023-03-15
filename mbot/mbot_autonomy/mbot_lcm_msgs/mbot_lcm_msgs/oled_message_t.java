/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package mbot_lcm_msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class oled_message_t implements lcm.lcm.LCMEncodable
{
    public long utime;
    public String line1;
    public String line2;
 
    public oled_message_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xfcdfd38929ceee80L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(mbot_lcm_msgs.oled_message_t.class))
            return 0L;
 
        classes.add(mbot_lcm_msgs.oled_message_t.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        char[] __strbuf = null;
        outs.writeLong(this.utime); 
 
        __strbuf = new char[this.line1.length()]; this.line1.getChars(0, this.line1.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
        __strbuf = new char[this.line2.length()]; this.line2.getChars(0, this.line2.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0); 
 
    }
 
    public oled_message_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public oled_message_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static mbot_lcm_msgs.oled_message_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        mbot_lcm_msgs.oled_message_t o = new mbot_lcm_msgs.oled_message_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        char[] __strbuf = null;
        this.utime = ins.readLong();
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.line1 = new String(__strbuf);
 
        __strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); this.line2 = new String(__strbuf);
 
    }
 
    public mbot_lcm_msgs.oled_message_t copy()
    {
        mbot_lcm_msgs.oled_message_t outobj = new mbot_lcm_msgs.oled_message_t();
        outobj.utime = this.utime;
 
        outobj.line1 = this.line1;
 
        outobj.line2 = this.line2;
 
        return outobj;
    }
 
}


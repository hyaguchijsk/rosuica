#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import nfc

card_rel = False

def released(tag):
    global card_rel
    print 'card released.'
    card_rel = True
    return True

def connected(tag):
    print tag
    ## Type3Tag == Felica
    if not isinstance(tag, nfc.tag.tt3.Type3Tag):
        print 'mismatch tag type'
        return
    print 'found Felica'

    ## 0x0003のsystem codeを持っていることが条件
    if not 0x0003 in tag.request_system_code():
        print 'Suica system code is not found'
        return
    print 'found Suica'

    idm, pmm = tag.polling(system_code = 0x0003)
    tag.idm, tag.pmm, tag.sys = idm, pmm, 0x0003
    print tag

    ## 0x090F だが逆順に入れないとダメっぽい
    sc = nfc.tag.tt3.ServiceCode.unpack(b'\x0F\x09')
    print sc

    for i in range(20):
        print 'No.: %2d' % i
        bc = nfc.tag.tt3.BlockCode(i, service = 0)
        data = tag.read_without_encryption([sc], [bc])

        ## http://www.wdic.org/w/RAIL/%E3%82%B5%E3%82%A4%E3%83%90%E3%83%8D%E8%A6%8F%E6%A0%BC%20%28IC%E3%82%AB%E3%83%BC%E3%83%89%29#xE4xB8xBBxE3x81xAAxE3x82xB5xE3x83xBCxE3x83x93xE3x82xB9xE3x82xB3xE3x83xBCxE3x83x89
        ## 0: 機器種別
        print '  Vendor: %02x' % data[0]
        ## 1: 利用種別
        print '  Usage: %02x' % data[1]
        ## 2: 決済種別
        print '  Settlement: %02x' % data[2]
        ## 3: 入出場種別
        print '  Enter/Exit: %02x' % data[3]
        ## 4-5: 年月日 7 + 4 + 5 bitsA
        date_msb = data[4]
        date_lsb = data[5]
        year = (date_msb >> 1) + 2000
        month = ((date_msb & 0x1) << 3) + (date_lsb >> 5)
        day = date_lsb & 31
        print '  Date: %04d/%02d/%02d' % (year, month, day)
        ## 6-9: 入出場もしくは停留所もしくは物販情報
        if data[1] == 0x46:  # 物販
            ## 6-7: 時刻 = H/5, M/6, (S/2)/5bits
            ts_msb = data[6]
            ts_lsb = data[7]
            hour = ts_msb >> 3
            minute = (ts_msb & 7) + (ts_lsb >> 5)
            second = (ts_lsb & 31) * 2
            print '    TimeStamp: %02d:%02d:%02d' % (hour, minute, second)
        ### 10-11: 残額
        print '  Balance: %5d' % ((data[11] << 8) + data[10])
        ### 12: Unknown
        ### 13-14: 履歴番号
        print '  Total No.: %2d' % ((data[13] << 8) + data[14])
        ### 15: 地域コード
        print '  Region: %02x' % data[15]
    return True

if __name__ == '__main__':
    clf = nfc.ContactlessFrontend('usb')

    try:
        while True:
            global card_rel
            card_rel = False
            ## 末尾FがFelicaになる
            print 'WAITING FOR CARD...'
            clf.connect(rdwr = {'targets' : {'424F', '212F'},
                                'on-release': released,
                                'on-connect': connected})
            while not card_rel:
                time.sleep(0.5)

    except KeyboardInterrupt:
        sys.exit()

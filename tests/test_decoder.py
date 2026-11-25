import binascii, csv, io, struct, tempfile, unittest, sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from decode_sweeps import HEADER, PREFIX, decode
class DecoderTest(unittest.TestCase):
    def test_valid_and_corrupt(self):
        self.assertEqual(HEADER.size,64); self.assertEqual(PREFIX.size,24)
        n=175
        h=HEADER.pack(b'RF247BIN',1,64,7,99,1,n,433050000,10000,12500,5000,2500,4,1,1,0,0,0,201,1,b'\0'*5)
        r=PREFIX.pack(0xA55A,1,1500,0,900,0,0,n)+bytes([200,255]+[190]*(n-2))
        r+=struct.pack('<H',binascii.crc_hqx(r,0xffff))
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/'test.bin'; f.write_bytes(h+r+r[:-1]+bytes([r[-1]^1])+b'bad')
            o=io.StringIO(); self.assertEqual(decode(f,o),2); rows=list(csv.DictReader(io.StringIO(o.getvalue())))
            self.assertEqual(len(rows),n); self.assertEqual(rows[0]['rssi_dbm'],'-100.0'); self.assertEqual(rows[1]['valid'],'0'); self.assertEqual(rows[-1]['frequency_hz'],'434790000')
if __name__=='__main__': unittest.main()

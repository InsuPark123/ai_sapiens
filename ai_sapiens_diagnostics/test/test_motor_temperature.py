import unittest
from ai_sapiens_diagnostics.motor_temperature import MotorTemperatures, MOTORS, INTERFACES


class TemperatureContract(unittest.TestCase):
    def setUp(self):
        self.cache = MotorTemperatures('k1-test')

    def source(self, sequence=1, count=1, **fields):
        row=dict(producer_session='session', publication_sequence=str(sequence), sample_count=str(count),
                 sample_age_ms='0', diagnostics_schema='2', sample_fresh='true',
                 sample_received='true', read_dhi_result='0')
        row.update(fields)
        return row

    def prime(self):
        self.cache = MotorTemperatures('k1-test')
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(),now=10)
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(2,201),now=10.2)
        self.cache.ingest_dynamic([('dxl1',INTERFACES,[38,35])],stamp=1,now=10.3)

    def test_value_units_identity_and_no_invented_devices(self):
        self.prime();rows=self.cache.snapshot(10.4)
        self.assertEqual(len(rows),23);self.assertEqual(sum(r['level']==0 for r in rows),1)
        row=rows[0];self.assertEqual(row['fields']['motor_temperature_c'],'35')
        self.assertEqual(row['fields']['inverter_temperature_c'],'38')
        self.assertEqual(row['fields']['unit'],'degC');self.assertEqual(row['fields']['robot_id'],'k1-test')

    def test_cache_without_advancing_dhi_proof_is_stale(self):
        self.cache.ingest_dynamic([('dxl1',INTERFACES,[38,35])],1,now=10)
        self.assertEqual(self.cache.snapshot(10)[0]['level'],3)
        self.prime()
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(3,201),now=10.4)
        self.assertEqual(self.cache.snapshot(10.4)[0]['level'],3)

    def test_expiry_and_packet_error_cannot_be_hidden_by_broadcaster(self):
        self.prime();self.assertEqual(self.cache.snapshot(11)[0]['level'],3)
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(3,401,read_dhi_result='-3002'),now=10.4)
        self.cache.ingest_dynamic([('dxl1',INTERFACES,[38,35])],2,now=10.5)
        self.assertEqual(self.cache.snapshot(10.5)[0]['level'],3)

    def test_generation_change_requires_new_data_and_progress(self):
        self.prime()
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(producer_session='reboot'),now=10.4)
        self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(2,201,producer_session='reboot'),now=10.5)
        self.assertEqual(self.cache.snapshot(10.5)[0]['level'],3)
        self.cache.ingest_dynamic([('dxl1',INTERFACES,[39,36])],2,now=10.6)
        self.assertEqual(self.cache.snapshot(10.6)[0]['level'],0)

    def test_replayed_diagnostics_cannot_extend_freshness(self):
        self.prime()
        self.assertFalse(self.cache.ingest_source('k1_right_arm/dxl/1/1',self.source(2,201),now=10.7))
        self.cache.ingest_dynamic([('dxl1',INTERFACES,[38,35])],2,now=10.9)
        self.assertEqual(self.cache.snapshot(10.9)[0]['level'],3)

    def test_malformed_or_partial_dynamic_does_not_keep_old_live_data(self):
        for rows in [[('dxl1',INTERFACES,[38,float('nan')])], [('dxl1',INTERFACES,[38,35])]*2,
                     [('dxl1',('Present Temperature',),[38])], [('dxl99',INTERFACES,[38,35])]]:
            self.prime();self.assertFalse(self.cache.ingest_dynamic(rows,3,now=10.4))
            self.assertEqual(self.cache.snapshot(10.4)[0]['level'],3)
        self.prime();self.cache.ingest_dynamic([],4,now=10.4)
        self.assertTrue(all(r['level']==3 for r in self.cache.snapshot(10.4)))

    def test_invalid_source_invalidates_current_value(self):
        for fields in [self.source(3,401,sample_age_ms='nan'),self.source(3,401,sample_fresh='false')]:
            self.prime();self.cache.ingest_source('k1_right_arm/dxl/1/1',fields,now=10.4)
            self.assertEqual(self.cache.snapshot(10.4)[0]['level'],3)

if __name__=='__main__':unittest.main()

"""Head-view crop regressions; generated images, no ROS or hardware."""
import copy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from robot.jetson.perception.family_store import FamilyStore
from robot.jetson.perception.wardrobe_tracking import WardrobeTracker,descriptor,similarity,appearance_bands,partial_view_similarity


class PartialClothingTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.store=FamilyStore(Path(self.tmp.name)/'family.sqlite3')
        self.mom=self.store.save_profile(dict(label='Mom',samples=[dict(embedding=[1.,0.])]))
        self.tracker=WardrobeTracker(self.store,'unused');self.addCleanup(self.tracker.close)
        self.tracker.retry_at=float('inf')
        self.full=np.zeros((240,100,3),np.uint8)
        self.full[:120]=(150,50,20);self.full[120:210]=(210,210,210);self.full[210:]=(90,130,190)
        self.lower=self.full[120:].copy()
        face=dict(box=[20,0,80,35],confirmed=True,profile_id=self.mom['profile_id'],revision=self.mom['revision'])
        self.tracker.observe(self.full,[[0,0,100,240]],[face],['cam',1,0],1.)
        self.original=self.tracker.selected(self.mom['profile_id'],1.)
        self.tracker.tracks[self.original['id']]['range']={'distance_m':.6}

    def lower_frame(self,now):
        self.tracker.observe(self.lower,[[0,0,100,120]],[],['cam',int(now),int(now%1*1e9)],now)
        return self.tracker.selected(self.mom['profile_id'],now)

    def test_partial_view_reacquires_twice_without_drifting_reference_or_range(self):
        self.assertLess(similarity(descriptor(self.full),descriptor(self.lower)),.78)
        before=copy.deepcopy(self.original['reference_descriptor'])
        bands=copy.deepcopy(self.original['reference_bands'])
        self.tracker.invalidate_position()
        self.assertIsNone(self.lower_frame(2.))
        result=self.lower_frame(2.2)
        self.assertEqual(self.original['id'],result['id'])
        self.assertEqual([0,0,100,120],result['box'])
        self.assertEqual('tracking',result['identity_source'])
        self.assertIsNone(result['range'])
        for n in range(5):result=self.lower_frame(2.3+n*.1)
        self.assertEqual(before,result['reference_descriptor']);self.assertEqual(bands,result['reference_bands'])
        self.assertFalse(self.store.wardrobe())
        self.tracker.invalidate_position()
        self.assertIsNone(self.lower_frame(4.))
        self.assertIsNotNone(self.lower_frame(4.2))

    def test_shared_trousers_do_not_choose_between_two_known_people(self):
        other=self.store.save_profile(dict(label='Other',samples=[dict(embedding=[0.,1.])]))
        competitor=copy.deepcopy(self.original)
        competitor.update(id='other-track',profile_id=other['profile_id'],revision=other['revision'])
        self.tracker.tracks['other-track']=competitor
        self.tracker.invalidate_position()
        self.assertIsNone(self.lower_frame(2.))
        self.assertIsNone(self.lower_frame(2.2))

    def test_two_visible_people_do_not_inherit_partial_identity(self):
        self.tracker.invalidate_position()
        image=np.concatenate([self.lower,self.lower],axis=1)
        for now in (2.,2.2):
            self.tracker.observe(image,[[0,0,100,120],[100,0,200,120]],[],['cam',2,int(now%1*1e9)],now)
            self.assertIsNone(self.tracker.selected(self.mom['profile_id'],now))

    def test_dark_trousers_conflict_and_long_gap_needs_new_identification(self):
        dark=self.lower.copy();dark[:90]=25
        self.assertLess(partial_view_similarity(appearance_bands(dark),appearance_bands(self.full)),.9)
        self.tracker.invalidate_position()
        self.assertIsNone(self.lower_frame(12.))
        self.assertIsNone(self.lower_frame(12.2))

    def test_face_override_wins_over_partial_clothing_continuity(self):
        self.tracker.invalidate_position();self.lower_frame(2.);self.lower_frame(2.2)
        other=self.store.save_profile(dict(label='Other',samples=[dict(embedding=[0.,1.])]))
        face=dict(box=[20,0,80,30],confirmed=True,profile_id=other['profile_id'],revision=other['revision'])
        self.tracker.observe(self.lower,[[0,0,100,120]],[face],['cam',3,0],3.)
        self.assertIsNone(self.tracker.selected(self.mom['profile_id'],3.))
        self.assertEqual('face',self.tracker.selected(other['profile_id'],3.)['identity_source'])

    def test_lost_positions_use_the_unweighted_appearance_margin(self):
        # Recorded failure: .984 vs .900 became only .0588 apart after weighting.
        competitor=copy.deepcopy(self.original)
        competitor.update(id='extra-body-part',profile_id=None,revision=None)
        self.tracker.tracks[competitor['id']]=competitor
        self.tracker.invalidate_position()
        with patch('robot.jetson.perception.wardrobe_tracking.similarity',side_effect=[.984,.900]):
            self.tracker.observe(self.full,[[0,0,100,240]],[],['cam',2,0],2.)
        target=self.tracker.selected(self.mom['profile_id'],2.)
        self.assertEqual(self.original['id'],target['id'])
        self.assertEqual('tracking',target['identity_source'])
        self.assertIsNone(target['range'])

    def test_close_appearance_alternatives_remain_ambiguous_after_movement(self):
        competitor=copy.deepcopy(self.original)
        competitor.update(id='similar-other-person',profile_id=None,revision=None)
        self.tracker.tracks[competitor['id']]=competitor
        self.tracker.invalidate_position()
        with patch('robot.jetson.perception.wardrobe_tracking.similarity',side_effect=[.984,.950]):
            self.tracker.observe(self.full,[[0,0,100,240]],[],['cam',2,0],2.)
        self.assertIsNone(self.tracker.selected(self.mom['profile_id'],2.))

    def test_missing_spatial_evidence_does_not_boost_stale_candidate_over_visible_track(self):
        competitor=copy.deepcopy(self.original)
        competitor.update(id='stale-candidate',profile_id=None,revision=None,position_valid=False,epoch=-1)
        self.tracker.tracks[competitor['id']]=competitor
        with patch('robot.jetson.perception.wardrobe_tracking.similarity',side_effect=[.984,.970]):
            self.tracker.observe(self.full,[[0,0,100,240]],[],['cam',2,0],2.)
        self.assertEqual(self.original['id'],self.tracker.selected(self.mom['profile_id'],2.)['id'])


if __name__=='__main__':unittest.main()

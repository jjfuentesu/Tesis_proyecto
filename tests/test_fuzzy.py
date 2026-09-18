import os
import pytest
from core.fuzzy_tuner import FuzzyTuner

def test_fuzzy_tuner_zero_error():
    tuner = FuzzyTuner("config/fuzzy_rules.json")
    dkp, dki, dkd = tuner.evaluate(0.0, 0.0)
    assert abs(dkp) < 0.05
    assert abs(dki) < 0.05
    assert abs(dkd) < 0.05

def test_fuzzy_tuner_max_error():
    tuner = FuzzyTuner("config/fuzzy_rules.json")
    dkp, dki, dkd = tuner.evaluate(1.0, 1.0)
    assert dkp != 0.0

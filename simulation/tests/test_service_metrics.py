import math
import pytest
from simulation.service.metrics import AlignedEvaluation


def test_service_metrics_remove_error_due_to_truth_age():
    metric=AlignedEvaluation()
    metric.estimate(.90,.90,0.,0.,100.)
    metric.estimate(1.,1.,0.,0.,100.)
    metric.evaluate(.96,.96,0.,0.,100.)
    result=metric.result(100.01)
    assert result["evaluation_aligned"]
    assert result["position_error_m"]==pytest.approx(0.)
    assert result["evaluation_comparison_time"]==.96
    assert result["evaluation_alignment_lag_ms"]==pytest.approx(40.)
    assert abs(1.-.96)>.03 # La comparación anterior inflaba el error hasta40mm.


def test_service_metrics_interpolate_truth_and_wrap_heading():
    metric=AlignedEvaluation()
    metric.evaluate(.9,.9,0.,math.radians(179),100.)
    metric.evaluate(1.,1.,0.,math.radians(-179),100.)
    metric.estimate(.95,.95,0.,math.pi,100.)
    result=metric.result(100.)
    assert result["position_error_m"]==pytest.approx(0.)
    assert result["heading_error_deg"]==pytest.approx(0.,abs=1e-8)


def test_service_metrics_no_extrapolation_or_stale_accuracy():
    metric=AlignedEvaluation()
    metric.evaluate(.8,0,0,0,100.)
    metric.estimate(.9,0,0,0,100.)
    assert not metric.result(100.)["evaluation_aligned"]
    metric.evaluate(.9,0,0,0,100.)
    assert metric.result(100.)["evaluation_aligned"]
    assert metric.result(100.31)["position_error_m"] is None


def test_service_metrics_reset_clears_previous_world_history():
    metric=AlignedEvaluation()
    metric.estimate(5.,1,0,0,100.)
    metric.evaluate(5.,1,0,0,100.)
    metric.evaluate(0.,0,0,0,101.)
    assert not metric.estimated
    assert not metric.result(101.)["evaluation_aligned"]
    metric.estimate(0.,0,0,0,101.)
    assert metric.result(101.)["position_error_m"]==0.

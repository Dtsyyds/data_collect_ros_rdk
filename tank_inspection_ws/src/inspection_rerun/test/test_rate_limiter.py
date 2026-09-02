from inspection_rerun.timestamp_utils import RateLimiter


def test_rate_limiter_and_rollback():
    limiter = RateLimiter(2.0)
    assert limiter.allow(1_000_000_000)
    assert not limiter.allow(1_100_000_000)
    assert limiter.allow(1_500_000_000)
    assert limiter.allow(100)  # replay restarted
    assert limiter.skipped == 1

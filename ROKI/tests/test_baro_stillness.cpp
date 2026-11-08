#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <initializer_list>
#include "baro_stillness.h"

static BaroStillness replay(float vertical_speed, float noise = 0.f) {
  BaroStillness trend{};
  float filtered = 0.f;
  for (uint32_t i = 0; i < 240; ++i) {
    const float seconds = i * .05f;
    const float raw = vertical_speed * seconds + noise *
        (sinf(i * 1.7f) + .4f * cosf(i * .41f));
    filtered = .82f * filtered + .18f * raw;
    trend.add(filtered, i * 50000UL);
  }
  return trend;
}

int main() {
  const BaroStillness standing = replay(0.f, .16f);
  assert(standing.ready());
  assert(standing.stable());
  for (float v : {-.5f, -.2f, -.15f, -.1f, -.05f,
                  .05f, .1f, .15f, .2f, .5f}) {
    const BaroStillness moving = replay(v, .16f);
    assert(moving.ready());
    assert(!moving.stable());
    assert(fabsf(moving.slopeMps() - v) < .02f);
  }
  BaroStillness wrap{};
  float filtered = 0.f;
  for (uint32_t i = 0; i < 240; ++i) {
    filtered = .82f * filtered + .18f * .1f;
    wrap.add(filtered, UINT32_MAX - 2000000UL + i * 50000UL);
  }
  assert(wrap.ready() && wrap.stable());
  puts("baro_stationarity: PASS");
  return 0;
}

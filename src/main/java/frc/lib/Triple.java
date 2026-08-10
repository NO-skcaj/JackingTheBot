// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib;

public class Triple<A, B, C> {
  private final A first;
  private final B second;
  private final C third;

  public Triple(A first, B second, C third) {
    this.first = first;
    this.second = second;
    this.third = third;
  }

  public A getFirst() {
    return first;
  }

  public B getSecond() {
    return second;
  }

  public C getThird() {
    return third;
  }
}

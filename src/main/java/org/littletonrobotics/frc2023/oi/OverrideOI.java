// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.oi;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for the override switches on the OI console. */
public class OverrideOI {
  private CommandGenericHID overrides;

  /** Creates a dummy set of overrides if controller is not available. */
  public OverrideOI() {}

  /** Creates a set of overrides using the given controller port. */
  public OverrideOI(int port) {
    overrides = new CommandGenericHID(port);
  }

  public Trigger setHigh() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(1);
  }

  public Trigger setMid() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(2);
  }

  public Trigger setLow() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(3);
  }

  public Trigger setColumn1() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(4);
  }

  public Trigger setColumn2() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(5);
  }

  public Trigger setColumn3() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(6);
  }

  public Trigger setColumn4() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(7);
  }

  public Trigger setColumn5() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(8);
  }

  public Trigger setColumn6() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(9);
  }

  public Trigger setColumn7() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(10);
  }

  public Trigger setColumn8() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(11);
  }

  public Trigger setColumn9() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.button(12);
  }

  public Trigger cubeButton() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.axisLessThan(0, -0.1);
  }

  public Trigger coneButton() {
    if (overrides == null) {
      return new Trigger(() -> false);
    }
    return overrides.axisLessThan(1, -0.1);
  }
}

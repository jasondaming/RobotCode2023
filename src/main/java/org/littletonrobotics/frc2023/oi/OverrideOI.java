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

  public Trigger setGridLeft() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(1);
  }

  public Trigger setGridCenter() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(2);
  }

  public Trigger setGridRight() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(3);
  }

  public Trigger setPositionUL() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(4);
  }

  public Trigger setPositionUC() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(5);
  }

  public Trigger setPositionUR() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(6);
  }

    public Trigger setPositionML() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(7);
  }

  public Trigger setPositionMC() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(8);
  }

  public Trigger setPositionMR() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(9);
  }

    public Trigger setPositionBL() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(10);
  }

  public Trigger setPositionBC() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(11);
  }

  public Trigger setPositionBR() {
    if (overrides == null) {
      return new Trigger();
    }
    return overrides.button(12);
  }
}

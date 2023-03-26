// OpenSTA, Static Timing Analyzer
// Copyright (c) 2023, Parallax Software, Inc.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include "Vector.hh"
#include "LibertyClass.hh"

namespace sta {

class WireloadForArea;

typedef std::pair<float,float> FanoutValue;
typedef Vector<FanoutValue*> FanoutValueSeq;
typedef Vector<WireloadForArea*> WireloadForAreaSeq;

const char *
wireloadTreeString(WireloadTree tree);
WireloadTree
stringWireloadTree(const char *tree);

const char *
wireloadModeString(WireloadMode wire_load_mode);
WireloadMode
stringWireloadMode(const char *wire_load_mode);

class Wireload
{
public:
  Wireload(const char *name,
	   LibertyLibrary *library,
     bool is_wireload_table);
  Wireload(const char *name,
	   LibertyLibrary *library,
	   float area,
	   float slope);
  virtual ~Wireload();
  const char *name() const { return name_; }
  void setArea(float area);
  void setSlope(float slope);
  void addFanoutLength(float fanout,
		       float length);
  void addFanoutCapacitance(float fanout,
		       float capacitance);
  void addFanoutResistance(float fanout,
		       float resistance);
  // Find wireload resistance/capacitance for fanout.
  virtual void findWireload(float fanout,
			    const OperatingConditions *op_cond,
			    float &cap,
			    float &res) const;
private:
  void addFanoutValue(float fanout, float value, FanoutValueSeq& fanout_values);
  float extractFanoutValue(float fanout, const FanoutValueSeq& fanout_values) const;

protected:
  const char *name_;
  LibertyLibrary *library_;
  bool is_wireload_table_;
  float area_;
  // Fanout length extrapolation slope.
  float slope_;
  FanoutValueSeq fanout_lengths_;
  FanoutValueSeq fanout_capacitances_;
  FanoutValueSeq fanout_resistances_;
};

class WireloadSelection
{
public:
  explicit WireloadSelection(const char *name);
  ~WireloadSelection();
  const char *name() const { return name_; }
  void addWireloadFromArea(float min_area,
			   float max_area,
			   const Wireload *wireload);
  const Wireload *findWireload(float area) const;

private:
  const char *name_;
  WireloadForAreaSeq wireloads_;
};

} // namespace

/*
 * Copyright (c) 2019 alpha group, CS department, University of Torino.
 *
 * This file is part of gam
 * (see https://github.com/alpha-unito/gam).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @brief       implements View class
 *
 * @ingroup internals
 *
 * @todo consider partial embedding of view tables into pointers
 *
 */
#ifndef INCLUDE_GAM_VIEW_HPP_
#define INCLUDE_GAM_VIEW_HPP_

#include <mutex>
#include <sstream>
#include <unordered_map>

#include "gam/ConcurrentMapWrap.hpp"
#include "gam/GlobalPointer.hpp"
#include "gam/backend_ptr.hpp"
#include "gam/defs.hpp"

namespace gam {

/**
 * View represents the global memory state as perceived by a single executor.
 *
 * @brief global memory as perceived by a single executor
 */
class View {
 public:
  /*
   ***************************************************************************
   *
   * constructors/destructor block
   *
   ***************************************************************************
   */
  ~View() {
    for (auto it = view_map.begin(); it != view_map.end(); ++it) {
      /* check no spurious committed copies */
      assert(it->second.committed == nullptr);
    }
  }

  /*
   ***************************************************************************
   *
   * getters block
   *
   ***************************************************************************
   */
  inline backend_ptr *committed(const uint64_t a) {
    return view_map[a].committed;
  }

  inline executor_id owner(const uint64_t a) { return view_map[a].owner; }

  inline void *child(const uint64_t a) { return view_map[a].child; }

  /*
   ***************************************************************************
   *
   * testers block
   *
   ***************************************************************************
   */
  inline bool has_child(const uint64_t a) {
    return (view_map[a].child != nullptr);
  }

  inline bool mapped(const uint64_t a) {
    return (view_map.find(a) != view_map.end());
  }

  /*
   ***************************************************************************
   *
   * setters block
   *
   ***************************************************************************
   */
  inline void bind_committed(const uint64_t a, backend_ptr *const p) {
    view_map[a].committed = p;
    LOGLN("VW  bind committed: %llu -> %p", a, p);
  }

  inline void bind_owner(const uint64_t a, const executor_id o) {
    view_map[a].owner = o;
    LOGLN("VW  bind owner: %llu -> %lu", a, o);
  }

  inline void bind_child(const uint64_t a, void *const c) {
    view_map[a].child = c;
    LOGLN("VW  bind child: %llu -> %p", a, c);
  }

  /*
   ***************************************************************************
   *
   * un-setters block
   *
   ***************************************************************************
   */

  inline void unmap(const uint64_t a) {
    assert(view_map.find(a) != view_map.end());

    view_map.erase(a);

    LOGLN("VW  cleared record=%llu", a);
  }

  /*
   ***************************************************************************
   *
   * misc block
   *
   ***************************************************************************
   */
  std::string to_string(const uint64_t a) {
    std::stringstream s;
    s << "(committed=" << committed(a) << " owner=" << owner(a)
      << " child=" << child(a) << ")";
    return s.str();
  }

 private:
  struct entry {
    backend_ptr *committed = nullptr;
    void *child = nullptr;
    executor_id owner;
  };

  ConcurrentMapWrap<std::unordered_map<uint64_t, entry>> view_map;
};

} /* namespace gam */

#endif /* INCLUDE_GAM_VIEW_HPP_ */

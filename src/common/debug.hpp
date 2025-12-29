#ifndef DEBUG_HPP_
#define DEBUG_HPP_

/*
 * Copyright (C) 2025 IEC Lab, DAA, NCKU
 *
 *     Author : Rong He
 *    Contact : P48101021@gs.ncku.edu.tw
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdexcept>

// 不要用assert,因為assert在release mode中無法工作
// 要用inline不然會有重複引入的問題
inline void ensure(bool cond, const char* msg) {
	if(!cond)
	{
			throw std::runtime_error(std::string("[ERROR]") + " " + msg);
	}
}
#endif  // DEBUG_HPP_
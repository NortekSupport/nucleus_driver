from __future__ import annotations

import sys
import time
from typing import Literal, Optional

C_MIRRORED = "\u2184"  # mirrored c, used in one direction only
Mode = Literal["auto", "tty", "log"]
SuffixMode = Literal["default", "bytes"]


class NucleusProgressBar:
    def __init__(
        self,
        *,
        stream=None,
        mode: Mode = "auto",
        min_interval_s: float = 0.25,
        min_percent_step: int = 1,
        min_iteration_step: int = 0,
        suffix: SuffixMode = "default",
    ) -> None:
        self.previous_pos = 0
        self.direction = 1
        self.stream = stream if stream is not None else sys.stdout
        self.mode: Mode = mode
        self.suffix_mode: SuffixMode = suffix

        self.min_interval_s = float(min_interval_s)
        self.min_percent_step = int(min_percent_step)
        self.min_iteration_step = int(min_iteration_step)

        self._last_emit_t: float = 0.0
        self._last_emit_percent: Optional[int] = None
        self._last_emit_iteration: Optional[int] = None
        self._start_t: Optional[float] = None

    def reset(self) -> None:
        self.previous_pos = 0
        self.direction = 1
        self._last_emit_t = 0.0
        self._last_emit_percent = None
        self._last_emit_iteration = None
        self._start_t = None

    def _effective_mode(self) -> Mode:
        if self.mode != "auto":
            return self.mode
        try:
            return "tty" if self.stream.isatty() else "log"
        except Exception:
            return "log"

    @staticmethod
    def _percent_int(iteration: int, total: int) -> int:
        if total <= 0:
            return 100 if iteration > 0 else 0
        if iteration <= 0:
            return 0
        if iteration >= total:
            return 100
        return int((iteration * 100) / total)

    def _should_emit_log(self, iteration: int, total: int) -> bool:
        now = time.monotonic()
        if self._last_emit_iteration is None:
            return True
        if total > 0 and iteration >= total:
            return True
        if self.min_interval_s > 0 and (now - self._last_emit_t) < self.min_interval_s:
            return False
        if self.min_iteration_step > 0:
            if (iteration - (self._last_emit_iteration or 0)) < self.min_iteration_step:
                return False
        if self.min_percent_step > 0:
            p = self._percent_int(iteration, total)
            last_p = self._last_emit_percent
            if last_p is not None and (p - last_p) < self.min_percent_step:
                return False
        return True

    def _mark_emitted(self, iteration: int, total: int) -> None:
        self._last_emit_t = time.monotonic()
        self._last_emit_iteration = iteration
        self._last_emit_percent = self._percent_int(iteration, total)

    def display_loading_bar(
        self,
        iteration: int,
        total: int,
        *,
        length: int = 30,
        head: str = "D",
        show_counts: bool = True,
        done: bool | None = None,
    ) -> None:
        if length <= 0:
            return

        if total <= 0:
            total = 1
        iteration = max(0, min(iteration, total))
        if self._start_t is None:
            self._start_t = time.monotonic()

        eff_mode = self._effective_mode()
        if done is None:
            done = (iteration >= total)

        if eff_mode == "log":
            if not self._should_emit_log(iteration, total):
                self._advance_sprite_state(iteration, total, length)
                return

        percent = iteration / float(total)

        is_final = (iteration >= total)

        if is_final:
            pos = length - 1  # only used for bounce logic / consistency
        else:
            pos = int(length * percent)
            if pos >= length:
                pos = length - 1

        pad = 3
        buf_len = length + (2 * pad) + 3
        bar = [" "] * buf_len

        relative_pos = self._step_sprite(pos=pos, length=length)

        sprite_start = pad + relative_pos
        head_pos = pad + pos

        sprite = f"-{C_MIRRORED})" if self.direction == 1 else "(c-"
        for i, ch in enumerate(sprite):
            x = sprite_start + i
            if 0 <= x < buf_len:
                bar[x] = ch

        if is_final:
            # Fill the entire visible bar with '-'; no head marker.
            for i in range(pad, pad + length):
                bar[i] = "-"
        else:
            for i in range(pad, head_pos):
                bar[i] = "-"
            bar[head_pos] = head

        visible = "".join(bar[pad : pad + length])
        bar_str = f"[{visible}]"
        suffix = self._build_suffix(iteration, total, show_counts=show_counts)

        if eff_mode == "tty":
            end = "\n" if done else "\r"
            self.stream.write(f"{bar_str}{suffix}{end}")
            self.stream.flush()
        else:
            self.stream.write(f"{bar_str}{suffix}\n")
            self.stream.flush()
            self._mark_emitted(iteration, total)

    def _step_sprite(self, *, pos: int, length: int) -> int:

        right_blank = length

        if self.direction == 1:
            relative_pos = self.previous_pos + 1
            self.previous_pos = relative_pos
            if relative_pos > right_blank:
                self.direction = -1
        else:
            relative_pos = self.previous_pos - 1
            self.previous_pos = relative_pos
            if relative_pos + 2 <= pos:
                self.direction = 1
        return relative_pos

    def _advance_sprite_state(self, iteration: int, total: int, length: int) -> None:
        if total <= 0:
            total = 1
        iteration = max(0, min(iteration, total))
        percent = iteration / float(total)
        pos = int(length * percent)
        if pos >= length:
            pos = length - 1
        self._step_sprite(pos=pos, length=length)

    def _build_suffix(self, iteration: int, total: int, *, show_counts: bool) -> str:
        if self.suffix_mode == "bytes":
            percent = self._percent_int(iteration, total)
            start_time = self._start_t or time.monotonic()
            elapsed = max(time.monotonic() - start_time, 1e-6)
            rate = iteration / elapsed
            return f" {percent}% {self._format_rate(rate)}"

        if show_counts:
            return f" {iteration}/{total}"
        return ""

    @staticmethod
    def _format_rate(rate: float) -> str:
        units = ["B/s", "KB/s", "MB/s", "GB/s", "TB/s"]
        value = float(rate)
        for unit in units:
            if abs(value) < 1024.0 or unit == units[-1]:
                return f"{value:.1f} {unit}"
            value /= 1024.0
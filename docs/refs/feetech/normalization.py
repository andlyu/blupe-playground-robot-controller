# Reference: installed lerobot/motors/motors_bus.py; lerobot 0.5.1; captured 2026-09-13
#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# Excerpt: MotorsBus normalization methods; reference only.
    def _normalize(self, ids_values: dict[int, int]) -> dict[int, float]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        normalized_values = {}
        for id_, val in ids_values.items():
            motor = self._id_to_name(id_)
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            bounded_val = min(max_, max(min_, val))
            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
                normalized_values[id_] = -norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                norm = ((bounded_val - min_) / (max_ - min_)) * 100
                normalized_values[id_] = 100 - norm if drive_mode else norm
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                normalized_values[id_] = (val - mid) * 360 / max_res
            else:
                raise NotImplementedError

        return normalized_values

    def _unnormalize(self, ids_values: dict[int, float]) -> dict[int, int]:
        if not self.calibration:
            raise RuntimeError(f"{self} has no calibration registered.")

        unnormalized_values = {}
        for id_, val in ids_values.items():
            motor = self._id_to_name(id_)
            min_ = self.calibration[motor].range_min
            max_ = self.calibration[motor].range_max
            drive_mode = self.apply_drive_mode and self.calibration[motor].drive_mode
            if max_ == min_:
                raise ValueError(f"Invalid calibration for motor '{motor}': min and max are equal.")

            if self.motors[motor].norm_mode is MotorNormMode.RANGE_M100_100:
                val = -val if drive_mode else val
                bounded_val = min(100.0, max(-100.0, val))
                unnormalized_values[id_] = int(((bounded_val + 100) / 200) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.RANGE_0_100:
                val = 100 - val if drive_mode else val
                bounded_val = min(100.0, max(0.0, val))
                unnormalized_values[id_] = int((bounded_val / 100) * (max_ - min_) + min_)
            elif self.motors[motor].norm_mode is MotorNormMode.DEGREES:
                mid = (min_ + max_) / 2
                max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                unnormalized_values[id_] = int((val * max_res / 360) + mid)
            else:
                raise NotImplementedError

        return unnormalized_values

    @abc.abstractmethod
    def _encode_sign(self, data_name: str, ids_values: dict[int, int]) -> dict[int, int]:
        pass

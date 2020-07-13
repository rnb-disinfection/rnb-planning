# Copyright 2018 The TensorFlow Constrained Optimization Authors. All Rights
# Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.
# ==============================================================================
"""A library for performing constrained optimization in TensorFlow."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from .python.candidates import find_best_candidate_distribution
from .python.candidates import find_best_candidate_index
from .python.constrained_minimization_problem import ConstrainedMinimizationProblem
from .python.rates.binary_rates import accuracy_rate
from .python.rates.binary_rates import error_rate
from .python.rates.binary_rates import f_score
from .python.rates.binary_rates import f_score_ratio
from .python.rates.binary_rates import false_negative_proportion
from .python.rates.binary_rates import false_negative_rate
from .python.rates.binary_rates import false_positive_proportion
from .python.rates.binary_rates import false_positive_rate
from .python.rates.binary_rates import inverse_precision_at_recall
from .python.rates.binary_rates import negative_prediction_rate
from .python.rates.binary_rates import positive_prediction_rate
from .python.rates.binary_rates import pr_auc
from .python.rates.binary_rates import precision
from .python.rates.binary_rates import precision_at_recall
from .python.rates.binary_rates import precision_ratio
from .python.rates.binary_rates import recall_at_precision
from .python.rates.binary_rates import roc_auc
from .python.rates.binary_rates import true_negative_proportion
from .python.rates.binary_rates import true_negative_rate
from .python.rates.binary_rates import true_positive_proportion
from .python.rates.binary_rates import true_positive_rate
from .python.rates.loss import BinaryClassificationLoss
from .python.rates.loss import HingeLoss
from .python.rates.loss import Loss
from .python.rates.loss import SoftmaxCrossEntropyLoss
from .python.rates.loss import SoftmaxLoss
from .python.rates.loss import ZeroOneLoss
from .python.rates.operations import lower_bound
from .python.rates.operations import upper_bound
from .python.rates.operations import wrap_rate
from .python.rates.rate_minimization_problem import RateMinimizationProblem
from .python.rates.subsettable_context import rate_context
from .python.rates.subsettable_context import split_rate_context
from .python.train.constrained_optimizer import ConstrainedOptimizerV1
from .python.train.constrained_optimizer import ConstrainedOptimizerV2
from .python.train.lagrangian_optimizer import create_lagrangian_loss
from .python.train.lagrangian_optimizer import LagrangianOptimizerV1
from .python.train.lagrangian_optimizer import LagrangianOptimizerV2
from .python.train.proxy_lagrangian_optimizer import create_proxy_lagrangian_loss
from .python.train.proxy_lagrangian_optimizer import ProxyLagrangianOptimizerV1
from .python.train.proxy_lagrangian_optimizer import ProxyLagrangianOptimizerV2

# The "true positive rate" is the same thing as the "recall", so we allow it to
# be accessed by either name.
recall = true_positive_rate

# By default, we use V2 optimizers. These aliases are purely for convenience: in
# general, you should prefer to explicitly specify either a V1 or a V2 optimizer
# (in case there's ever a V3, in which case we'll update these aliases).
ConstrainedOptimizer = ConstrainedOptimizerV2
LagrangianOptimizer = LagrangianOptimizerV2
ProxyLagrangianOptimizer = ProxyLagrangianOptimizerV2

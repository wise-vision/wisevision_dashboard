/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import { e2e } from '@grafana/e2e';

e2e.scenario({
  describeName: 'Smoke test',
  itName: 'Smoke test',
  scenario: () => {
    e2e.pages.Home.visit();
    e2e().contains('Welcome to Grafana').should('be.visible');
  },
});

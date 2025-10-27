/*
 * Copyright (C) 2025 wisevision
 *
 * SPDX-License-Identifier: MPL-2.0
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

import React from 'react';
import { Route, Switch } from 'react-router-dom';
import { AppRootProps } from '@grafana/data';
import { ROUTES } from '../../constants';
import { PageOne, PageAIAgent } from '../../pages';
import { prefixRoute } from '../../utils/utils.routing';

export function App(props: AppRootProps) {
  return (
    <Switch>
      <Route exact path={prefixRoute(ROUTES.AIAgent)} component={PageAIAgent} />

      {/* Full-width page (this page will have no side navigation) */}

      {/* Default page */}
      <Route component={PageOne} />
    </Switch>
  );
}

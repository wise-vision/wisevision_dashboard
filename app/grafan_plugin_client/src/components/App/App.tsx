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

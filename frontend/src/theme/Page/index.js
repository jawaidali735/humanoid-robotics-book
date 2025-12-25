import React from 'react';
import Page from '@theme-original/Page';
import ProtectedDocsContent from '../../components/ProtectedDocsContent';

export default function PageWrapper(props) {
  const isDocsPath = props.location?.pathname?.startsWith('/docs/');

  if (isDocsPath) {
    return (
      <ProtectedDocsContent path={props.location?.pathname} title={props.title}>
        <Page {...props} />
      </ProtectedDocsContent>
    );
  }

  return <Page {...props} />;
}
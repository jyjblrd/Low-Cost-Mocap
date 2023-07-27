import React from 'react';
import Button from 'react-bootstrap/Button';

export default function Toolbar() {
  return (
    <div style={{ float: 'right' }}>
      <Button
        variant="outline-dark"
        className="mx-1"
      >
        Save
      </Button>
    </div>
  );
}
import React, { useState } from "react";
import { Fab } from '@material-ui/core';
import { Button } from '@material-ui/core';
import { Dialog } from '@material-ui/core';
import { DialogActions } from '@material-ui/core';
import { DialogTitle } from '@material-ui/core';
import { demoLink } from '../links';
import NavigationIcon from '@material-ui/icons/Navigation';

export const Home = () => {
    const [open, setOpen] = useState(false);

    const handleClose = () => {
        setOpen(false);
    }

    const handleClick = () => {
        const data = {};
        console.log(JSON.stringify(data));

        fetch(demoLink
            , {
                method: 'POST',
                mode: 'cors',
                headers: {
                    'Accept': 'application/json, text/plain, */*',
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(data)
            }
        ).then(response => {
            return response.json()
        }).then(responseJSON => {
            if(responseJSON.success){
                setOpen(true)
            }
        })
        .catch(e=>console.log(e))
    }

    return (
        <div style={{height: "100%", display: "flex", justifyContent: "center", alignItems: "center"}}>
            <Fab variant="extended" size="large" style={{height: 'auto', width: '400px', padding: '50px'}} onClick={handleClick}>
                <NavigationIcon />
                Start Tour
            </Fab>
            <Dialog open={open} onClose={handleClose} aria-labelledby="form-dialog-title">
            <DialogTitle id="form-dialog-title">Tour has been initiated</DialogTitle>
        <DialogActions>
          <Button onClick={handleClose} color="primary">
            Close
          </Button>
        </DialogActions>
      </Dialog>
        </div>
    );
}
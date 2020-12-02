import React, {useState} from "react";
import Fab from '@material-ui/core/Fab';
import {Button, Dialog, DialogActions, DialogTitle} from '@material-ui/core'
import NavigationIcon from '@material-ui/icons/Navigation';
import {tourLink} from '../links'

export default function Home() {
    const [open, setOpen] = useState<boolean>(false);

    const handleClose = () => {
        setOpen(false);
    }

    const handleClick = () => {
        const data = {start_tour: true};
        console.log(JSON.stringify(data))

        fetch(tourLink
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